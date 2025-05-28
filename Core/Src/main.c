/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "memorymap.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h" // Inclua o cabeçalho do Timer configurado (ex: tim.h para TIM1)

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h> // Para sprintf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_TIMER_HANDLE    htim1 // Altere para o handle do seu Timer (ex: htim1, htim2, etc.)
#define PWM_TIMER_CHANNEL   TIM_CHANNEL_1 // Altere para o canal PWM usado (ex: TIM_CHANNEL_1, TIM_CHANNEL_2, etc.)
#define PWM_PERIOD          (__HAL_TIM_GET_AUTORELOAD(&PWM_TIMER_HANDLE) + 1) // Obtém o ARR (Auto-Reload Register) do Timer
#define USER_BUTTON_Pin     B1_Pin // Assumindo que o botão de usuário é o B1 (Blue Button na NUCLEO-F429ZI)
#define USER_BUTTON_GPIO_Port B1_GPIO_Port // Porta GPIO do botão
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_data;
uint8_t current_duty_cycle = 0; // Variável para armazenar o duty cycle atual para a opção 4
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Set_PWM_Duty(uint8_t duty_percent);
/* USER CODE BEGIN PFP */
void Display_Menu(void);
void ramp_pwm(void);
void handle_button_increment(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Redireciona printf para a UART
/*int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}*/
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
  MX_USART3_UART_Init();
  MX_TIM1_Init(); // Inicialize o Timer configurado (ex: MX_TIM1_Init();)
  /* USER CODE BEGIN 2 */

  // Inicia o PWM
  HAL_TIM_PWM_Start(&PWM_TIMER_HANDLE, PWM_TIMER_CHANNEL);
  Set_PWM_Duty(0); // Inicia com o PWM em 0%
  printf(">> PWM iniciado em 0%%\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Display_Menu();

    // Aguarda entrada do usuário
    if (HAL_UART_Receive(&huart3, &rx_data, 1, HAL_MAX_DELAY) == HAL_OK)
    {
      switch (rx_data)
      {
        case '1':
          printf(">> Duty fixo 20%% selecionado.\r\n");
          Set_PWM_Duty(20);
          break;

        case '2':
          printf(">> Duty fixo 80%% selecionado.\r\n");
          Set_PWM_Duty(80);
          break;

        case '3':
          printf(">> Ramp 0-100%% em 5s selecionado.\r\n");
          ramp_pwm();
          break;

        case '4':
          printf(">> Incremento de 10%% por clique no botao. Pressione o botao de usuario.\r\n");
          handle_button_increment();
          break;

        default:
          printf(">> Opcao invalida. Tente novamente.\r\n");
          break;
      }
    }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief Exibe o menu de opções via UART.
  * @retval None
  */
void Display_Menu(void) {
    printf("\r\n"); // Nova linha para melhor formatação
    printf("===== CONTROLE DE PWM =====\r\n");
    printf("[1] Duty fixo 20%%\r\n");
    printf("[2] Duty fixo 80%%\r\n");
    printf("[3] Ramp 0-100%% em 5 s\r\n");
    printf("[4] +10%% por clique no botao\r\n");
    printf("Selecione a opcao: ");
}

/**
  * @brief Ajusta o duty cycle do PWM.
  * @param duty_percent: O duty cycle em porcentagem (0-100).
  * @retval None
  */
void Set_PWM_Duty(uint8_t duty_percent) {
    if (duty_percent > 100) {
        duty_percent = 100;
    }
    // Calcula o valor do registrador de comparação (CCR)
    // O valor do CCR é duty_percent * PWM_PERIOD / 100
    uint32_t ccr_value = (uint32_t)((float)PWM_PERIOD * duty_percent / 100.0f);

    __HAL_TIM_SET_COMPARE(&PWM_TIMER_HANDLE, PWM_TIMER_CHANNEL, ccr_value);
    current_duty_cycle = duty_percent; // Atualiza o duty cycle atual
}

/**
  * @brief Implementa uma rampa de PWM de 0% a 100% em 5 segundos.
  * @retval None
  */
void ramp_pwm(void) {
    uint16_t delay_ms = 5000 / 101; // 5 segundos (5000 ms) dividido por 101 passos (0 a 100)
    for (uint8_t duty = 0; duty <= 100; duty++) {
        Set_PWM_Duty(duty);
        HAL_Delay(delay_ms);
    }
    printf(">> Rampa concluida. Duty cycle em 100%%\r\n");
}

/**
  * @brief Lida com o incremento de 10% no duty cycle via botão.
  * @retval None
  */
void handle_button_increment(void) {
    uint8_t last_duty = current_duty_cycle;
    // Loop para esperar o botão ou uma nova entrada UART
    while (HAL_UART_Receive(&huart3, &rx_data, 1, 0) != HAL_OK) { // Non-blocking receive
        if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_RESET) { // Botão pressionado (assumindo pull-up)
            HAL_Delay(50); // Debounce
            while(HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_RESET); // Espera soltar o botão

            current_duty_cycle += 10;
            if (current_duty_cycle > 100) {
                current_duty_cycle = 0;
            }
            Set_PWM_Duty(current_duty_cycle);
            printf(">> Duty atual: %d%%\r\n", current_duty_cycle);
        }
        // Pequeno delay para evitar loop muito rápido
        HAL_Delay(10);
    }
    // Se uma nova entrada UART for recebida, saia da função para o loop principal
    // A variável rx_data já conterá o novo valor digitado.
    // Restaura o último duty cycle para a opção 4, caso não seja mantido pela próxima opção.
    if (rx_data != '4') {
        current_duty_cycle = last_duty;
    }
}

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


    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
