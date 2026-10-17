/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float x_cm;
    float y_cm;
    float theta_deg;      /* Rumbo continuo en grados */
    float theta_enc_deg;  /* Rumbo estimado por encoders */
    float vel_izq_cms;
    float vel_der_cms;
    float yaw_rate_dps;   /* Velocidad angular vertical en grados/s */
} Odometria_State_t;

typedef struct {
    volatile int32_t  posicion;   /* Pulsos acumulados CON signo (+ adelante, - atras) */
    volatile int8_t   sentido;    /* +1 / -1 segun el ultimo sentido comandado al motor */
    volatile uint32_t t_flanco;   /* Instante (ciclos DWT) del ultimo pulso valido */
} Encoder_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- DIMENSIONES Y ANGULOS DE LAS FIGURAS --- */
#define LADO_CUADRADO_CM       30.0f   /* 1 Clic:  Lado del cuadrado (cm) */
#define ANGULO_CUADRADO_DEG    90.0f   /* 1 Clic:  Angulo de giro en cada esquina del cuadrado */

#define LADO_TRIANGULO_CM      30.0f   /* 2 Clics: Lado del triangulo equilatero (cm) */
#define ANGULO_TRIANGULO_DEG   120.0f  /* 2 Clics: Angulo exterior de giro del triangulo */

#define RADIO_CIRCULO_CM       18.0f   /* 3 Clics: Circulo (radio del centro del carro) */
#define LADO_PENTAGONO_CM      22.0f   /* 4 Clics: Pentagono */
#define LADO_HEXAGONO_CM       20.0f   /* 5 Clics: Hexagono */
#define LADO_OCTAGONO_CM       15.0f   /* 6 Clics: Octagono */

/* =====================================================================
 *  PERILLAS DE CALIBRACION MANUAL  (ajusta aqui si deseas afinar)
 *  7 clics: TEST recta 100 cm      -> ajusta ESCALA_AVANCE y TRIM_DER_PWM
 *  8 clics: TEST 4 x 90 en sitio   -> ajusta ANTICIP_GIRO_DEG y ESCALA_GIRO
 *  9 clics: TEST 8 x 45 en sitio   -> ajusta ESCALA_GIRO
 *  10 clics: figura libre (tabla FIGURA_LIBRE mas abajo)
 * ===================================================================== */
#define ESCALA_AVANCE       1.0f   /* Si avanza de menos, subir (ej. 1.15f); si avanza de mas, bajar */
#define ANTICIP_AVANCE_CM   1.0f    /* Se pasa de la meta en recta -> subir; se queda corto -> bajar */
#define TRIM_DER_PWM        0       /* Se curva a la DERECHA -> subir (+10); a la IZQ -> bajar (-10) */
#define ESCALA_GIRO         1.47f /* 370/360: giraba 10 grados de mas por vuelta */  /* Si gira DE MAS -> SUBIR (ej. 1.30f); si gira DE MENOS -> BAJAR (ej. 0.85f) */
#define ANTICIP_GIRO_DEG    12.0f   /* Grados antes de la meta en que frena. Si se pasa -> subir */
#define AUTOAPRENDIZAJE     0       /* 0 = fijo. Con 1 la anticipacion cambia en cada prueba y no se puede calibrar */

/* --- PARAMETROS FISICOS DEL CHASIS --- */
#define DIAMETRO_RUEDA_CM   6.5f
#define ANCHO_LLANTA_CM     2.5f
#define DIST_INTERNA_CM     11.0f
#define ANCHO_VIA_CM        (DIST_INTERNA_CM + ANCHO_LLANTA_CM) /* 13.5 cm nominal */
#define RANURAS_ENCODER     20.0f
#define PI                  3.14159265f
#define RAD2DEG             (180.0f / PI)
#define DEG2RAD             (PI / 180.0f)
#define CIRCUNFERENCIA_CM   (PI * DIAMETRO_RUEDA_CM)
#define FLANCOS_POR_VUELTA  RANURAS_ENCODER
#define CM_POR_FLANCO       (CIRCUNFERENCIA_CM / FLANCOS_POR_VUELTA)

/* --- FILTRO ANTIRREBOTE DE ENCODERS OPTICOS (LM393) --- */
/* 4.5 ms filtra el rebote sin perder pulsos reales cuando la rueda gira rapido */
#define ENC_DEBOUNCE_US     4500U
#define ENC_QUIETO_MS       200U

/* --- PWM L298N (0..999) --- */
#define PWM_MIN_MOVIMIENTO  455
#define PWM_MAX_LIMITE      920
#define PWM_FRENO           999
#define PWM_AVANCE_BASE     590
#define PWM_AVANCE_LENTO    480
#define PWM_GIRO_ARRANQUE   590     /* Impulso corto y suave solo para despegar del reposo */
#define PWM_GIRO_BASE       525     /* Velocidad de giro moderada y controlada */
#define PWM_GIRO_LENTO      465     /* Aproximacion final lenta para frenar sin pasarse */
#define PWM_CIRCULO_BASE    580

/* --- PERFILES DE MOVIMIENTO --- */
#define PERIODO_CONTROL_US  5000U   /* Lazo de control fijo a 200 Hz (5 ms) */
#define RAMPA_AVANCE_MS     200U
#define IMPULSO_GIRO_MS     35U     /* Solo 35 ms de despegue inicial */
#define DIST_FRENADO_CM     6.0f
#define ZONA_LENTA_GIRO_DEG 35.0f   /* Desacelera durante los ultimos 35 grados */
#define FRENO_AVANCE_MS     85U
#define FRENO_GIRO_MS       110U    /* Freno firme al terminar cada giro */
#define T_ASENTAR_MS        240U

/* --- GANANCIAS DE CONTROL --- */
#define KP_RUMBO            8.0f    /* Linea recta: P sobre error de rumbo */
#define KD_RUMBO            0.25f   /* Linea recta: D sobre velocidad angular (dps) */
#define KI_RUMBO            5.0f    /* Linea recta: I suave */
#define KP_SYNC_ENC         10.0f   /* Sincronizacion de pulsos si ambos encoders estan activos */
#define INTEG_RUMBO_MAX     45.0f
#define CORR_RUMBO_MAX      100.0f
#define KP_GIRO_SYNC        4.0f
#define KP_CIRC             8.0f
#define KD_CIRC             0.25f
#define KI_CIRC             8.0f
#define INTEG_CIRC_MAX      80.0f
#define CORR_CIRC_MAX       150.0f

/* --- APRENDIZAJE DE INERCIA ACOTADO --- */
#define ALPHA_APRENDIZAJE   0.25f

/* --- MPU6050 (INTEGRACION VECTORIAL 3D INVARIANTE A ORIENTACION) --- */
#define MPU6050_ADDR_LOW    (0x68 << 1)
#define MPU6050_ADDR_HIGH   (0x69 << 1)
#define GYRO_LSB_POR_DPS    32.8f   /* Escala +-1000 dps */
#define GYRO_RAW_SAT        32500
#define ALPHA_ZUPT          0.02f
#define ZUPT_UMBRAL_DPS     2.5f
#define DT_MAX_S            0.05f
#define TIMEOUT_I2C_MS      4U
#define I2C_VELOCIDAD_HZ    100000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static Encoder_t enc_izq = {0, 1, 0};
static Encoder_t enc_der = {0, 1, 0};
static int32_t pos_izq_prev = 0;
static int32_t pos_der_prev = 0;

static uint8_t  dwt_ok = 0;
static uint32_t ciclos_por_us = 72U;
static uint32_t t_estado_ciclos = 0;
static volatile uint8_t motores_activos = 0;
static volatile int8_t  dir_giro_activo = 0;

/* Offsets en reposo de los 3 ejes del giroscopio y vector unitario 3D del eje vertical */
static uint16_t mpu_addr = MPU6050_ADDR_LOW;
float gx_offset = 0.0f;
float gy_offset = 0.0f;
float gz_offset = 0.0f;
float eje_vert_x = 1.0f;
float eje_vert_y = 0.0f;
float eje_vert_z = 0.0f;
static uint8_t eje_gyro_aprendido = 0;

/* Ultimas velocidades angulares corregidas en los 3 ejes (dps) para integracion 3D en giros */
static float gx_corr_dps = 0.0f;
static float gy_corr_dps = 0.0f;
static float gz_corr_dps = 0.0f;
static float dt_ultimo_s = 0.005f;

uint8_t mpu_conectado = 0;
static uint8_t  gyro_fallo = 0;
static uint8_t  gyro_saturado = 0;
static float    bias_yaw_dps = 0.0f;
static float    w_prev_dps = 0.0f;
static uint8_t  errores_i2c_seguidos = 0;
static uint16_t recuperaciones_i2c = 0;

/* Parametros autoajustables acotados */
static float ancho_via_eff_cm  = ANCHO_VIA_CM;
static float anticip_giro_deg  = ANTICIP_GIRO_DEG;
static float anticip_avance_cm = ANTICIP_AVANCE_CM;
static float integ_rumbo       = 0.0f;
static float rumbo_ref_deg     = 0.0f;

Odometria_State_t robot_odom = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
uint16_t Limitar_PWM(int32_t valor_pwm, uint16_t min_pwm, uint16_t max_pwm);
void Motores_Set(int32_t pwm_izq, int32_t pwm_der);
void Motores_Detener(void);
void Motores_FrenoCorto(uint32_t ms);
void Motores_FrenoActivo(uint8_t sentido_previo);
void Motores_Avanzar(uint16_t pwm_izq, uint16_t pwm_der);
void Motores_Retroceder(uint16_t pwm_izq, uint16_t pwm_der);
void Motores_GirarDerecha(uint16_t pwm_izq, uint16_t pwm_der);
void Motores_GirarIzquierda(uint16_t pwm_izq, uint16_t pwm_der);

void MPU6050_Init(void);
uint8_t MPU6050_Calibrar(void);

void Estado_Actualizar(void);
void Odometria_Reiniciar(void);
void Avanzar_Distancia_cm(float distancia_cm);
void Girar_A_Rumbo(float rumbo_objetivo_deg);
void Girar_Grados(float grados_derecha);

void Hacer_Poligono_Regular(uint8_t num_lados, float lado_cm);
void Hacer_Cuadrado(float lado_cm);
void Hacer_Triangulo(float lado_cm);
void Hacer_Circulo(float radio_cm);
void Hacer_Pentagono(float lado_cm);
void Hacer_Hexagono(float lado_cm);
void Hacer_Octagono(float lado_cm);

uint8_t Leer_Selector_Boton(void);
void Parpadear_LED(uint8_t veces);
void Telemetria_Enviar(const char *etiqueta, int32_t indice);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ======================= 0. TIEMPO DE ALTA RESOLUCION ======================= */
static void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    ciclos_por_us = SystemCoreClock / 1000000U;
    uint32_t c0 = DWT->CYCCNT;
    for (volatile uint32_t i = 0; i < 200U; i++) { }
    dwt_ok = (DWT->CYCCNT != c0) ? 1U : 0U;
}

static inline uint32_t Ciclos(void) {
    return dwt_ok ? DWT->CYCCNT : (HAL_GetTick() * (SystemCoreClock / 1000U));
}

static void Retardo_us(uint32_t us) {
    uint32_t t0 = Ciclos();
    uint32_t c = us * ciclos_por_us;
    while ((Ciclos() - t0) < c) { }
}

static void Esperar_Periodo(uint32_t *t_sig) {
    uint32_t periodo = PERIODO_CONTROL_US * ciclos_por_us;
    while ((int32_t)(Ciclos() - *t_sig) < 0) { }
    *t_sig += periodo;
    if ((int32_t)(Ciclos() - *t_sig) > 0) {
        *t_sig = Ciclos() + periodo;
    }
}

static void Esperar_Activo(uint32_t ms) {
    uint32_t t0 = HAL_GetTick();
    uint32_t t_sig = Ciclos();
    while ((HAL_GetTick() - t0) < ms) {
        Esperar_Periodo(&t_sig);
        Estado_Actualizar();
    }
}

static float Angulo_Normalizar(float a) {
    while (a > 180.0f)   a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

static float Limitar_f(float v, float lim) {
    if (v > lim)  return lim;
    if (v < -lim) return -lim;
    return v;
}

static float Acotar_f(float v, float min_v, float max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static int32_t Redondear(float v) {
    return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/* ======================= 1. ENCODERS CON FILTRO ANTI-RUIDO ======================= */
static void Encoder_Flanco(Encoder_t *e, GPIO_TypeDef *puerto, uint16_t pin, uint32_t ahora) {
    if ((ahora - e->t_flanco) < (ENC_DEBOUNCE_US * ciclos_por_us)) return;

    /* Verifica que el pin permanezca en ALTO durante ~80 us para descartar picos de ruido */
    for (uint8_t muestra = 0; muestra < 3U; muestra++) {
        for (volatile uint32_t i = 0; i < 180U; i++) { }
        if (HAL_GPIO_ReadPin(puerto, pin) != GPIO_PIN_SET) return;
    }

    e->t_flanco = ahora;
    e->posicion += e->sentido;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint32_t ahora = Ciclos();
    if (GPIO_Pin == GPIO_PIN_6) {
        Encoder_Flanco(&enc_izq, GPIOA, GPIO_PIN_6, ahora);
    } else if (GPIO_Pin == GPIO_PIN_7) {
        Encoder_Flanco(&enc_der, GPIOA, GPIO_PIN_7, ahora);
    }
}

static void Encoders_Configurar_UnFlanco(void) {
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_IT_RISING;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);
    enc_izq.t_flanco = Ciclos();
    enc_der.t_flanco = Ciclos();
}

static uint8_t Encoders_Quietos(uint32_t ms) {
    uint32_t lim = ms * 1000U * ciclos_por_us;
    uint32_t ahora = Ciclos();
    return ((ahora - enc_izq.t_flanco) > lim) && ((ahora - enc_der.t_flanco) > lim);
}

/* Si uno de los dos encoders cuenta menos de la mitad que el otro, usa el encoder sano */
static float Distancia_Desde(int32_t pi0, int32_t pd0) {
    int32_t di = enc_izq.posicion - pi0;
    int32_t dd = enc_der.posicion - pd0;
    int32_t ai = abs(di);
    int32_t ad = abs(dd);
    if (ai > 3 && ad * 2 < ai) return (float)di * CM_POR_FLANCO;
    if (ad > 3 && ai * 2 < ad) return (float)dd * CM_POR_FLANCO;
    return ((float)di + (float)dd) * 0.5f * CM_POR_FLANCO;
}

/* ======================= 2. BOTON Y LED ======================= */
uint8_t Leer_Selector_Boton(void) {
    uint8_t clics = 0;
    uint32_t t_ultimo_clic = 0;

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {
        HAL_Delay(45);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {
            while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {
                HAL_Delay(10);
            }
            HAL_Delay(45);

            clics = 1;
            t_ultimo_clic = HAL_GetTick();

            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_Delay(60);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

            while ((HAL_GetTick() - t_ultimo_clic) < 1800U) {
                if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {
                    HAL_Delay(45);
                    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {
                        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {
                            HAL_Delay(10);
                        }
                        HAL_Delay(45);

                        clics++;
                        t_ultimo_clic = HAL_GetTick();

                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                        HAL_Delay(60);
                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                    }
                }
            }
        }
    }
    return clics;
}

void Parpadear_LED(uint8_t veces) {
    for (uint8_t i = 0; i < veces; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(200);
    }
}

/* ======================= 3. PUENTE H L298N ======================= */
uint16_t Limitar_PWM(int32_t valor_pwm, uint16_t min_pwm, uint16_t max_pwm) {
    if (valor_pwm < (int32_t)min_pwm) return min_pwm;
    if (valor_pwm > (int32_t)max_pwm) return max_pwm;
    return (uint16_t)valor_pwm;
}

void Motores_Set(int32_t pwm_izq, int32_t pwm_der) {
    if (pwm_izq > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        enc_izq.sentido = 1;
    } else if (pwm_izq < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        enc_izq.sentido = -1;
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);
    }

    if (pwm_der > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        enc_der.sentido = 1;
    } else if (pwm_der < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        enc_der.sentido = -1;
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
    }

    uint32_t ci = (uint32_t)abs(pwm_izq);
    uint32_t cd = (uint32_t)abs(pwm_der);
    if (ci > PWM_MAX_LIMITE) ci = PWM_MAX_LIMITE;
    if (cd > PWM_MAX_LIMITE) cd = PWM_MAX_LIMITE;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ci);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, cd);
    motores_activos = (pwm_izq != 0 || pwm_der != 0) ? 1U : 0U;
}

void Motores_Detener(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    motores_activos = 0;
}

void Motores_FrenoCorto(uint32_t ms) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_FRENO);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_FRENO);
    motores_activos = 1;
    Esperar_Activo(ms);
    Motores_Detener();
}

void Motores_Avanzar(uint16_t pwm_izq, uint16_t pwm_der)        { Motores_Set(pwm_izq, pwm_der); }
void Motores_Retroceder(uint16_t pwm_izq, uint16_t pwm_der)     { Motores_Set(-(int32_t)pwm_izq, -(int32_t)pwm_der); }
void Motores_GirarDerecha(uint16_t pwm_izq, uint16_t pwm_der)   { Motores_Set(pwm_izq, -(int32_t)pwm_der); }
void Motores_GirarIzquierda(uint16_t pwm_izq, uint16_t pwm_der) { Motores_Set(-(int32_t)pwm_izq, pwm_der); }
void Motores_FrenoActivo(uint8_t sentido_previo) { (void)sentido_previo; Motores_FrenoCorto(FRENO_AVANCE_MS); }

/* ======================= 4. MPU-6050 VERTICAL 3D (INVARIANTE A ORIENTACION) ======================= */
static void I2C_Recuperar(void) {
    HAL_I2C_DeInit(&hi2c1);

    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_Init(GPIOB, &g);

    for (uint8_t i = 0; i < 9U; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        Retardo_us(10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        Retardo_us(10);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    Retardo_us(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    Retardo_us(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    Retardo_us(10);

    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    Retardo_us(20);
    __HAL_RCC_I2C1_RELEASE_RESET();
    Retardo_us(20);

    hi2c1.Init.ClockSpeed = I2C_VELOCIDAD_HZ;
    HAL_I2C_Init(&hi2c1);
}

static uint8_t MPU6050_Escribir(uint8_t reg, uint8_t valor) {
    return (HAL_I2C_Mem_Write(&hi2c1, mpu_addr, reg, 1, &valor, 1, 20) == HAL_OK) ? 1U : 0U;
}

void MPU6050_Init(void) {
    uint8_t check = 0;
    mpu_conectado = 0;

    const uint16_t dirs[2] = {MPU6050_ADDR_LOW, MPU6050_ADDR_HIGH};
    for (uint8_t intento = 0; intento < 3U; intento++) {
        for (uint8_t d = 0; d < 2U; d++) {
            mpu_addr = dirs[d];
            if (HAL_I2C_Mem_Read(&hi2c1, mpu_addr, 0x75, 1, &check, 1, 25) == HAL_OK &&
                (check != 0x00 && check != 0xFF)) {
                MPU6050_Escribir(0x6B, 0x80);
                HAL_Delay(100);
                uint8_t ok = MPU6050_Escribir(0x6B, 0x01);
                ok &= MPU6050_Escribir(0x19, 0x00);
                ok &= MPU6050_Escribir(0x1A, 0x03); /* DLPF 44 Hz */
                ok &= MPU6050_Escribir(0x1B, 0x10); /* Giroscopio +-1000 dps (32.8 LSB/dps) */
                ok &= MPU6050_Escribir(0x1C, 0x00);
                if (ok) {
                    mpu_conectado = 1;
                    gyro_fallo = 0;
                    recuperaciones_i2c = 0;
                    HAL_Delay(50);
                    return;
                }
            }
        }
        I2C_Recuperar();
        HAL_Delay(25);
    }
}

/* Calibra los offsets en reposo de los 3 ejes del giroscopio */
uint8_t MPU6050_Calibrar(void) {
    if (!mpu_conectado) return 0;

    uint8_t buf[6];
    float sg[3] = {0.0f, 0.0f, 0.0f};
    uint16_t n = 0;

    for (uint16_t i = 0; i < 200U; i++) {
        if (HAL_I2C_Mem_Read(&hi2c1, mpu_addr, 0x43, 1, buf, 6, 10) == HAL_OK) {
            int16_t rx = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t ry = (int16_t)((buf[2] << 8) | buf[3]);
            int16_t rz = (int16_t)((buf[4] << 8) | buf[5]);
            sg[0] += (float)rx / GYRO_LSB_POR_DPS;
            sg[1] += (float)ry / GYRO_LSB_POR_DPS;
            sg[2] += (float)rz / GYRO_LSB_POR_DPS;
            n++;
        }
        HAL_Delay(2);
    }
    if (n < 100U) {
        I2C_Recuperar();
        return 0;
    }

    gx_offset = sg[0] / (float)n;
    gy_offset = sg[1] / (float)n;
    gz_offset = sg[2] / (float)n;
    bias_yaw_dps = 0.0f;
    w_prev_dps = 0.0f;
    gyro_fallo = 0;
    return 1U;
}

static uint8_t MPU6050_LeerGyro3D(float *w_proj) {
    uint8_t buf[6];
    if (HAL_I2C_Mem_Read(&hi2c1, mpu_addr, 0x43, 1, buf, 6, TIMEOUT_I2C_MS) != HAL_OK) {
        if (++errores_i2c_seguidos >= 3U) {
            errores_i2c_seguidos = 0;
            I2C_Recuperar();
            if (++recuperaciones_i2c > 60U) gyro_fallo = 1;
            gx_corr_dps = 0.0f;
            gy_corr_dps = 0.0f;
            gz_corr_dps = 0.0f;
        }
        return 0;
    }
    errores_i2c_seguidos = 0;

    int16_t rx = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ry = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t rz = (int16_t)((buf[4] << 8) | buf[5]);
    gyro_saturado = (abs(rx) > GYRO_RAW_SAT || abs(ry) > GYRO_RAW_SAT || abs(rz) > GYRO_RAW_SAT) ? 1U : 0U;

    float gx = (((float)rx / GYRO_LSB_POR_DPS) - gx_offset) * ESCALA_GIRO;
    float gy = (((float)ry / GYRO_LSB_POR_DPS) - gy_offset) * ESCALA_GIRO;
    float gz = (((float)rz / GYRO_LSB_POR_DPS) - gz_offset) * ESCALA_GIRO;

    /* En reposo corrige lentamente cualquier deriva termica en los 3 ejes */
    uint8_t quieto = (!motores_activos) && Encoders_Quietos(ENC_QUIETO_MS);
    if (quieto && fabsf(gx) < ZUPT_UMBRAL_DPS && fabsf(gy) < ZUPT_UMBRAL_DPS && fabsf(gz) < ZUPT_UMBRAL_DPS) {
        gx_offset += ALPHA_ZUPT * (gx / ESCALA_GIRO);
        gy_offset += ALPHA_ZUPT * (gy / ESCALA_GIRO);
        gz_offset += ALPHA_ZUPT * (gz / ESCALA_GIRO);
        gx = 0.0f;
        gy = 0.0f;
        gz = 0.0f;
    }

    gx_corr_dps = gx;
    gy_corr_dps = gy;
    gz_corr_dps = gz;

    if (eje_gyro_aprendido) {
        *w_proj = (gx * eje_vert_x) + (gy * eje_vert_y) + (gz * eje_vert_z);
    } else if (dir_giro_activo != 0) {
        float mag = sqrtf(gx * gx + gy * gy + gz * gz);
        *w_proj = (mag > 1.5f) ? ((float)dir_giro_activo * mag) : 0.0f;
    } else {
        *w_proj = 0.0f;
    }
    return 1;
}

/* ======================= 5. ESTIMADOR DE ESTADO ======================= */
void Estado_Actualizar(void) {
    uint32_t ahora = Ciclos();
    float dt = (float)(ahora - t_estado_ciclos) / (float)SystemCoreClock;
    if (dt <= 0.0f) return;
    t_estado_ciclos = ahora;
    if (dt > DT_MAX_S) dt = DT_MAX_S;
    dt_ultimo_s = dt;

    int32_t pi = enc_izq.posicion;
    int32_t pd = enc_der.posicion;
    float dl = (float)(pi - pos_izq_prev) * CM_POR_FLANCO;
    float dr = (float)(pd - pos_der_prev) * CM_POR_FLANCO;
    pos_izq_prev = pi;
    pos_der_prev = pd;

    float ds = 0.5f * (dl + dr);
    float dth_enc = ((dr - dl) / ancho_via_eff_cm) * RAD2DEG;
    float dth = (mpu_conectado && !gyro_fallo) ? 0.0f : dth_enc;
    float rate = 0.0f;
    uint8_t usar_gyro = 0;

    if (mpu_conectado && !gyro_fallo) {
        float w_proj;
        if (MPU6050_LeerGyro3D(&w_proj)) {
            float dth_g = 0.5f * (w_proj + w_prev_dps) * dt;
            w_prev_dps = w_proj;

            if (!gyro_saturado && (eje_gyro_aprendido || dir_giro_activo != 0)) {
                dth = dth_g;
                rate = w_proj;
                usar_gyro = 1;
            }
        } else {
            w_prev_dps = 0.0f;
        }
    }

    float th_mid = (robot_odom.theta_deg + 0.5f * dth) * DEG2RAD;
    robot_odom.x_cm += ds * cosf(th_mid);
    robot_odom.y_cm += ds * sinf(th_mid);
    robot_odom.theta_deg += dth;
    robot_odom.theta_enc_deg += dth_enc;

    robot_odom.yaw_rate_dps = usar_gyro ? rate : (0.7f * robot_odom.yaw_rate_dps + 0.3f * rate);
    robot_odom.vel_izq_cms = 0.8f * robot_odom.vel_izq_cms + 0.2f * (dl / dt);
    robot_odom.vel_der_cms = 0.8f * robot_odom.vel_der_cms + 0.2f * (dr / dt);
}

static uint8_t Giro_Confiable(void) {
    return (mpu_conectado && !gyro_fallo) ? 1U : 0U;
}

void Odometria_Reiniciar(void) {
    Estado_Actualizar();
    robot_odom.x_cm = 0.0f;
    robot_odom.y_cm = 0.0f;
    robot_odom.theta_deg = 0.0f;
    robot_odom.theta_enc_deg = 0.0f;
    rumbo_ref_deg = 0.0f;
    integ_rumbo = 0.0f;
}

/* ======================= 6. MOVIMIENTOS REALIMENTADOS ======================= */
void Avanzar_Distancia_cm(float distancia_cm) {
    if (distancia_cm <= 0.0f) return;

    dir_giro_activo = 0;
    Estado_Actualizar();
    const int32_t pi0 = enc_izq.posicion;
    const int32_t pd0 = enc_der.posicion;
    const float dt_ctrl = (float)PERIODO_CONTROL_US * 1e-6f;

    /* Mantiene el rumbo IDEAL que fijo Hacer_Figura; solo lo descarta si es absurdo */
    if (fabsf(rumbo_ref_deg - robot_odom.theta_deg) > 20.0f || !eje_gyro_aprendido) {
        rumbo_ref_deg = robot_odom.theta_deg;
    }

    distancia_cm *= ESCALA_AVANCE;
    float meta_parada = distancia_cm - anticip_avance_cm;
    if (meta_parada < CM_POR_FLANCO) meta_parada = CM_POR_FLANCO;

    uint32_t t0 = HAL_GetTick();
    uint32_t timeout_ms = (uint32_t)(distancia_cm * 80.0f) + 1200U;
    uint32_t t_sig = Ciclos();
    float s = 0.0f;

    while (1) {
        Esperar_Periodo(&t_sig);
        Estado_Actualizar();
        s = Distancia_Desde(pi0, pd0);
        uint32_t tr = HAL_GetTick() - t0;
        if (s >= meta_parada || tr >= timeout_ms) break;

        float base = (float)PWM_AVANCE_BASE;
        if (tr < RAMPA_AVANCE_MS) {
            base = PWM_MIN_MOVIMIENTO + (PWM_AVANCE_BASE - PWM_MIN_MOVIMIENTO) * ((float)tr / RAMPA_AVANCE_MS);
        }
        float restante = meta_parada - s;
        if (restante < DIST_FRENADO_CM) {
            float b = PWM_AVANCE_LENTO + (PWM_AVANCE_BASE - PWM_AVANCE_LENTO) * (restante / DIST_FRENADO_CM);
            if (b < base) base = b;
        }

        float corr = 0.0f;
        if (eje_gyro_aprendido && Giro_Confiable()) {
            float e = Limitar_f(rumbo_ref_deg - robot_odom.theta_deg, 15.0f);
            integ_rumbo = Limitar_f(integ_rumbo + KI_RUMBO * e * dt_ctrl, INTEG_RUMBO_MAX);
            corr = Limitar_f(KP_RUMBO * e - KD_RUMBO * robot_odom.yaw_rate_dps + integ_rumbo,
                             CORR_RUMBO_MAX);
        } else {
            int32_t di = enc_izq.posicion - pi0;
            int32_t dd = enc_der.posicion - pd0;
            if (di > 3 && dd > 3 && di < 2 * dd && dd < 2 * di) {
                corr = Limitar_f((float)(di - dd) * KP_SYNC_ENC, 60.0f);
            }
        }

        Motores_Set(Limitar_PWM((int32_t)(base - corr), PWM_MIN_MOVIMIENTO, PWM_MAX_LIMITE),
                    Limitar_PWM((int32_t)(base + corr) + TRIM_DER_PWM, PWM_MIN_MOVIMIENTO, PWM_MAX_LIMITE));
    }

    float s_parada = s;
    Motores_FrenoCorto(FRENO_AVANCE_MS);
    Esperar_Activo(T_ASENTAR_MS);
    s = Distancia_Desde(pi0, pd0);

    float inercia = s - s_parada;
    if (AUTOAPRENDIZAJE && s_parada >= meta_parada && inercia >= 0.0f && inercia < 5.0f) {
        anticip_avance_cm = (1.0f - ALPHA_APRENDIZAJE) * anticip_avance_cm + ALPHA_APRENDIZAJE * inercia;
        anticip_avance_cm = Acotar_f(anticip_avance_cm, 0.3f, 3.0f);
    }
}

/* Gira un angulo relativo controlado a velocidad moderada (+ = derecha, - = izquierda) */
void Girar_Grados(float grados_derecha) {
    float meta_deg = fabsf(grados_derecha);
    if (meta_deg <= 2.0f) return;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    Estado_Actualizar();

    const float dir = (grados_derecha > 0.0f) ? -1.0f : 1.0f;
    dir_giro_activo = (grados_derecha > 0.0f) ? -1 : 1;

    const int32_t pi0 = enc_izq.posicion;
    const int32_t pd0 = enc_der.posicion;
    const float th0 = robot_odom.theta_deg;

    float anticip = anticip_giro_deg;
    if (anticip > 0.35f * meta_deg) anticip = 0.35f * meta_deg;
    float objetivo_parada = meta_deg - anticip;

    float ang_x = 0.0f;
    float ang_y = 0.0f;
    float ang_z = 0.0f;
    float girado_deg = 0.0f;
    float girado_prev_check = 0.0f;
    int32_t pulsos_prev_check = 0;
    float pwm_boost = 0.0f;

    uint32_t t0 = HAL_GetTick();
    uint32_t t_check = t0;
    uint32_t timeout_ms = (uint32_t)(meta_deg * 25.0f) + 900U;
    uint32_t t_sig = Ciclos();

    while (1) {
        Esperar_Periodo(&t_sig);
        Estado_Actualizar();

        ang_x += gx_corr_dps * dt_ultimo_s;
        ang_y += gy_corr_dps * dt_ultimo_s;
        ang_z += gz_corr_dps * dt_ultimo_s;
        float girado_gyro_3d = sqrtf(ang_x * ang_x + ang_y * ang_y + ang_z * ang_z);

        int32_t a_i = abs(enc_izq.posicion - pi0);
        int32_t a_d = abs(enc_der.posicion - pd0);
        int32_t pulsos_max = (a_i > a_d) ? a_i : a_d;
        int32_t pulsos_ref = (a_i > 2 && a_d > 2 && a_i < 2 * a_d && a_d < 2 * a_i)
                           ? ((a_i + a_d) / 2) : pulsos_max;
        float girado_enc = (((2.0f * (float)pulsos_ref * CM_POR_FLANCO) / ancho_via_eff_cm) * RAD2DEG) * ESCALA_GIRO;

        /* Usa la lectura mayor entre giroscopio 3D y encoders para nunca pasarse de giro */
        if (Giro_Confiable() && girado_gyro_3d >= girado_enc) {
            girado_deg = girado_gyro_3d;
        } else {
            girado_deg = girado_enc;
        }

        uint32_t ahora_ms = HAL_GetTick();
        uint32_t tr = ahora_ms - t0;
        if (girado_deg >= objetivo_parada || tr >= timeout_ms) break;

        /* Anti-atasco suave: solo actua si ni el giroscopio ni los encoders registran movimiento */
        if ((ahora_ms - t_check) >= 160U) {
            if ((girado_deg - girado_prev_check) < 2.0f && pulsos_max == pulsos_prev_check) {
                pwm_boost += 30.0f;
                if (pwm_boost > 110.0f) pwm_boost = 110.0f;
            } else {
                pwm_boost = 0.0f; /* Ya se esta moviendo: quita el empuje extra de inmediato */
            }
            girado_prev_check = girado_deg;
            pulsos_prev_check = pulsos_max;
            t_check = ahora_ms;
        }

        float restante = meta_deg - girado_deg;
        float base;
        if (tr < IMPULSO_GIRO_MS && meta_deg > 25.0f) {
            base = (float)PWM_GIRO_ARRANQUE;
        } else if (restante >= ZONA_LENTA_GIRO_DEG) {
            base = (float)PWM_GIRO_BASE;
        } else {
            base = PWM_GIRO_LENTO + (PWM_GIRO_BASE - PWM_GIRO_LENTO) * (restante / ZONA_LENTA_GIRO_DEG);
        }
        base += pwm_boost;

        int32_t diff = 0;
        if (a_i > 2 && a_d > 2 && a_i < 2 * a_d && a_d < 2 * a_i) {
            diff = a_i - a_d;
            if (diff > 4)  diff = 4;
            if (diff < -4) diff = -4;
        }

        int32_t m_i = Limitar_PWM((int32_t)(base - diff * KP_GIRO_SYNC), PWM_MIN_MOVIMIENTO, PWM_MAX_LIMITE);
        int32_t m_d = Limitar_PWM((int32_t)(base + diff * KP_GIRO_SYNC), PWM_MIN_MOVIMIENTO, PWM_MAX_LIMITE);
        Motores_Set((int32_t)(-dir) * m_i, (int32_t)dir * m_d);
    }

    float girado_parada = girado_deg;
    Motores_FrenoCorto(FRENO_GIRO_MS);

    uint32_t t_asentar0 = HAL_GetTick();
    uint32_t t_sig_a = Ciclos();
    while ((HAL_GetTick() - t_asentar0) < T_ASENTAR_MS) {
        Esperar_Periodo(&t_sig_a);
        Estado_Actualizar();
        ang_x += gx_corr_dps * dt_ultimo_s;
        ang_y += gy_corr_dps * dt_ultimo_s;
        ang_z += gz_corr_dps * dt_ultimo_s;
    }
    dir_giro_activo = 0;

    float girado_gyro_final = sqrtf(ang_x * ang_x + ang_y * ang_y + ang_z * ang_z);

    if (Giro_Confiable() && girado_gyro_final > 35.0f) {
        float ux = dir * (ang_x / girado_gyro_final);
        float uy = dir * (ang_y / girado_gyro_final);
        float uz = dir * (ang_z / girado_gyro_final);
        if (!eje_gyro_aprendido) {
            eje_vert_x = ux;
            eje_vert_y = uy;
            eje_vert_z = uz;
            eje_gyro_aprendido = 1U;
        } else {
            eje_vert_x = 0.80f * eje_vert_x + 0.20f * ux;
            eje_vert_y = 0.80f * eje_vert_y + 0.20f * uy;
            eje_vert_z = 0.80f * eje_vert_z + 0.20f * uz;
            float n = sqrtf(eje_vert_x * eje_vert_x + eje_vert_y * eje_vert_y + eje_vert_z * eje_vert_z);
            if (n > 0.1f) {
                eje_vert_x /= n;
                eje_vert_y /= n;
                eje_vert_z /= n;
            }
        }
        robot_odom.theta_deg = th0 + dir * girado_gyro_final;
    }

    float girado_final = (Giro_Confiable() && girado_gyro_final > girado_parada) ? girado_gyro_final : girado_parada;
    float inercia = girado_final - girado_parada;
    if (AUTOAPRENDIZAJE && girado_parada >= objetivo_parada && inercia >= 0.0f && inercia < 22.0f) {
        anticip_giro_deg = (1.0f - ALPHA_APRENDIZAJE) * anticip_giro_deg + ALPHA_APRENDIZAJE * inercia;
        anticip_giro_deg = Acotar_f(anticip_giro_deg, 2.0f, 18.0f);
    }

    rumbo_ref_deg = robot_odom.theta_deg;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

void Girar_A_Rumbo(float rumbo_objetivo_deg) {
    Estado_Actualizar();
    float e0 = Angulo_Normalizar(rumbo_objetivo_deg - robot_odom.theta_deg);
    Girar_Grados(-e0);

    /* Un solo retoque si quedo a mas de 4 grados del rumbo objetivo */
    Estado_Actualizar();
    float e1 = Angulo_Normalizar(rumbo_objetivo_deg - robot_odom.theta_deg);
    if (fabsf(e1) > 4.0f) {
        Girar_Grados(-e1);
    }
}

/* ======================= 7. FIGURAS GEOMETRICAS ======================= */
typedef struct {
    float avanzar_cm;
    float girar_der_deg;
} Paso_t;

/* FIGURA LIBRE (10 clics): edita esta tabla para dibujar lo que quieras */
static const Paso_t FIGURA_LIBRE[] = {
    {40.0f, 90.0f},
    {20.0f, 90.0f},
    {40.0f, 90.0f},
    {20.0f, 90.0f},
};

/* Rumbos ABSOLUTOS: si un vertice se pasa o se queda corto, el siguiente giro lo compensa
   y el error NO se acumula vuelta tras vuelta */
void Hacer_Figura(const Paso_t *pasos, uint8_t n) {
    Odometria_Reiniciar();
    float rumbo_obj = 0.0f;   /* + = izquierda (convencion de theta) */
    for (uint8_t i = 0; i < n; i++) {
        rumbo_ref_deg = rumbo_obj;
        Avanzar_Distancia_cm(pasos[i].avanzar_cm);
        if (pasos[i].girar_der_deg != 0.0f) {
            rumbo_obj -= pasos[i].girar_der_deg;
            Girar_A_Rumbo(rumbo_obj);
            rumbo_ref_deg = rumbo_obj;
        }
        Telemetria_Enviar("VERTICE", (int32_t)(i + 1U));
    }
}

void Hacer_Poligono_Regular(uint8_t num_lados, float lado_cm) {
    if (num_lados < 3U || num_lados > 12U) return;
    Paso_t p[12];
    for (uint8_t i = 0; i < num_lados; i++) {
        p[i].avanzar_cm = lado_cm;
        p[i].girar_der_deg = 360.0f / (float)num_lados;
    }
    Hacer_Figura(p, num_lados);
}

void Hacer_Cuadrado(float lado_cm) {
    Paso_t p[4];
    for (uint8_t i = 0; i < 4U; i++) {
        p[i].avanzar_cm = lado_cm;
        p[i].girar_der_deg = ANGULO_CUADRADO_DEG;
    }
    Hacer_Figura(p, 4U);
}

void Hacer_Triangulo(float lado_cm) {
    Paso_t p[3];
    for (uint8_t i = 0; i < 3U; i++) {
        p[i].avanzar_cm = lado_cm;
        p[i].girar_der_deg = ANGULO_TRIANGULO_DEG;
    }
    Hacer_Figura(p, 3U);
}

void Hacer_Pentagono(float lado_cm)  { Hacer_Poligono_Regular(5, lado_cm); }
void Hacer_Hexagono(float lado_cm)   { Hacer_Poligono_Regular(6, lado_cm); }
void Hacer_Octagono(float lado_cm)   { Hacer_Poligono_Regular(8, lado_cm); }

/* --- PRUEBAS DE CALIBRACION --- */
void Test_Recta(void) {                 /* 7 clics: debe recorrer 100 cm derecho */
    Odometria_Reiniciar();
    Avanzar_Distancia_cm(100.0f);
}

void Test_Giros(uint8_t n) {            /* 8 clics: 4x90, 9 clics: 8x45 */
    Odometria_Reiniciar();
    float paso_deg = 360.0f / (float)n;
    float rumbo_obj = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        rumbo_obj -= paso_deg;
        Girar_A_Rumbo(rumbo_obj);   /* Absoluto: el error que quede es SOLO de escala del giroscopio */
        Telemetria_Enviar("GIRO", (int32_t)(i + 1U));
        Esperar_Activo(300);
    }
}

void Hacer_Circulo(float radio_cm) {
    Odometria_Reiniciar();

    const float W = ancho_via_eff_cm;
    if (radio_cm < 0.6f * W) radio_cm = 0.6f * W;
    const float k_ext = (radio_cm + 0.5f * W) / radio_cm;
    const float k_int = (radio_cm - 0.5f * W) / radio_cm;
    const float perimetro = 2.0f * PI * radio_cm;
    const float meta = perimetro - anticip_avance_cm;
    const float th0 = robot_odom.theta_deg;
    const int32_t pi0 = enc_izq.posicion;
    const int32_t pd0 = enc_der.posicion;
    const float dt_ctrl = (float)PERIODO_CONTROL_US * 1e-6f;

    float integ = 0.0f;
    float ang_x = 0.0f, ang_y = 0.0f, ang_z = 0.0f;
    uint32_t t0 = HAL_GetTick();
    uint32_t t_sig = Ciclos();

    dir_giro_activo = -1;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    while (1) {
        Esperar_Periodo(&t_sig);
        Estado_Actualizar();
        ang_x += gx_corr_dps * dt_ultimo_s;
        ang_y += gy_corr_dps * dt_ultimo_s;
        ang_z += gz_corr_dps * dt_ultimo_s;

        float s = Distancia_Desde(pi0, pd0);
        uint32_t tr = HAL_GetTick() - t0;
        float girado_3d = sqrtf(ang_x * ang_x + ang_y * ang_y + ang_z * ang_z);
        if (s >= meta || girado_3d >= 360.0f || tr > 15000U) break;

        float base = (float)PWM_CIRCULO_BASE;
        if (tr < RAMPA_AVANCE_MS) {
            base = PWM_MIN_MOVIMIENTO + (PWM_CIRCULO_BASE - PWM_MIN_MOVIMIENTO) * ((float)tr / RAMPA_AVANCE_MS);
        }

        float th_actual = Giro_Confiable() ? (th0 - girado_3d) : robot_odom.theta_deg;
        float th_des = th0 - (s / radio_cm) * RAD2DEG;
        float v_c = 0.5f * (robot_odom.vel_izq_cms + robot_odom.vel_der_cms);
        float w_des = -(v_c / radio_cm) * RAD2DEG;
        float e = Limitar_f(th_des - th_actual, 25.0f);

        integ = Limitar_f(integ + KI_CIRC * e * dt_ctrl, INTEG_CIRC_MAX);
        float corr = Limitar_f(KP_CIRC * e + KD_CIRC * (w_des - robot_odom.yaw_rate_dps) + integ,
                               CORR_CIRC_MAX);

        float m_i = PWM_MIN_MOVIMIENTO + (base - PWM_MIN_MOVIMIENTO) * k_ext - corr;
        float m_d = PWM_MIN_MOVIMIENTO + (base - PWM_MIN_MOVIMIENTO) * k_int + corr;

        Motores_Set(Limitar_PWM((int32_t)m_i, PWM_MIN_MOVIMIENTO, PWM_MAX_LIMITE),
                    Limitar_PWM((int32_t)m_d, PWM_MIN_MOVIMIENTO - 30, PWM_MAX_LIMITE));
    }

    dir_giro_activo = 0;
    Motores_FrenoCorto(FRENO_AVANCE_MS);
    Esperar_Activo(T_ASENTAR_MS);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

/* ======================= 8. TELEMETRIA POR USART3 (115200) ======================= */
void Telemetria_Enviar(const char *etiqueta, int32_t indice) {
    char buf[200];
    int n = snprintf(buf, sizeof(buf),
        "%s,%ld,x_mm=%ld,y_mm=%ld,th_x10=%ld,thEnc_x10=%ld,W_x100=%ld,antG_x10=%ld,antA_mm=%ld,"
        "trim=%ld,bias_mdps=%ld,gyro=%u,eje=%u\r\n",
        etiqueta, (long)indice,
        (long)Redondear(robot_odom.x_cm * 10.0f),
        (long)Redondear(robot_odom.y_cm * 10.0f),
        (long)Redondear(robot_odom.theta_deg * 10.0f),
        (long)Redondear(robot_odom.theta_enc_deg * 10.0f),
        (long)Redondear(ancho_via_eff_cm * 100.0f),
        (long)Redondear(anticip_giro_deg * 10.0f),
        (long)Redondear(anticip_avance_cm * 10.0f),
        (long)Redondear(integ_rumbo),
        (long)Redondear(bias_yaw_dps * 1000.0f),
        (unsigned)Giro_Confiable(),
        (unsigned)eje_gyro_aprendido);
    if (n > 0) {
        if (n > (int)sizeof(buf)) n = (int)sizeof(buf);
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, 50);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* LED apagado */

  DWT_Init();
  Encoders_Configurar_UnFlanco();

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  Motores_Detener();

  /* Mantener el carro QUIETO al encender */
  HAL_Delay(500);
  MPU6050_Init();

  if (mpu_conectado) {
      MPU6050_Calibrar();
      Parpadear_LED(2);
  } else {
      Parpadear_LED(5);
  }
  t_estado_ciclos = Ciclos();
  pos_izq_prev = enc_izq.posicion;
  pos_der_prev = enc_der.posicion;
  Telemetria_Enviar("INICIO", (int32_t)mpu_conectado);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint8_t opcion = Leer_Selector_Boton();

      if (opcion > 0) {
          Parpadear_LED(opcion);
          HAL_Delay(1000);

          if (mpu_conectado) {
              MPU6050_Calibrar();
          }

          if (opcion == 1) {
              Hacer_Cuadrado(LADO_CUADRADO_CM);
          }
          else if (opcion == 2) {
              Hacer_Triangulo(LADO_TRIANGULO_CM);
          }
          else if (opcion == 3) {
              Hacer_Circulo(RADIO_CIRCULO_CM);
          }
          else if (opcion == 4) {
              Hacer_Pentagono(LADO_PENTAGONO_CM);
          }
          else if (opcion == 5) {
              Hacer_Hexagono(LADO_HEXAGONO_CM);
          }
          else if (opcion == 6) {
              Hacer_Octagono(LADO_OCTAGONO_CM);
          }
          else if (opcion == 7) {
              Test_Recta();
          }
          else if (opcion == 8) {
              Test_Giros(4);
          }
          else if (opcion == 9) {
              Test_Giros(8);
          }
          else {
              Hacer_Figura(FIGURA_LIBRE, (uint8_t)(sizeof(FIGURA_LIBRE) / sizeof(FIGURA_LIBRE[0])));
          }

          Telemetria_Enviar("FIN", (int32_t)opcion);
          Parpadear_LED(2);
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
