<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<Project>
  <Device>
    <Manufacturer id="ST" name="ST"/>
    <Chip id="STM32F103C8" name="STM32F103C8"/>
  </Device>
  <Components>
    <Component id="c37a9351-96ed-11df-80ae-001d7d723e56" name="C Library"/>
    <Component id="bf7c3c91-96ed-11df-80ae-001d7d723e56" name="CMSIS core"/>
    <Component id="bf8a1f41-96ed-11df-80ae-001d7d723e56" name="CMSIS_Boot"/>
    <Component id="bfa102a1-96ed-11df-80ae-001d7d723e56" name="RCC"/>
    <Component id="bfe42621-96ed-11df-80ae-001d7d723e56" name="GPIO"/>
    <Component id="bff208d1-96ed-11df-80ae-001d7d723e56" name="EXTI"/>
    <Component id="c0446e91-96ed-11df-80ae-001d7d723e56" name="SPI"/>
    <Component id="c04d9651-96ed-11df-80ae-001d7d723e56" name="I2C"/>
    <Component id="c064a0c1-96ed-11df-80ae-001d7d723e56" name="TIM"/>
  </Components>
  <ExcludedFiles>
    <Exclude path="stm32f10x_it.c"/>
    <Exclude path="Simple_Demo_Source/system_stm32f10x.c"/>
    <Exclude path="Simple_Demo_Source/stm32f10x_conf.h"/>
    <Exclude path="Simple_Demo_Source/stm32f10x_it.c"/>
    <Exclude path="Simple_Demo_Source/TC.c"/>
    <Exclude path="synhron_prot.c"/>
    <Exclude path="synhron_prot.h"/>
    <Exclude path="my_task.c"/>
    <Exclude path="my_task.h"/>
    <Exclude path="BASE_SYSTEM_STM_FUN/P_config_dev/check_init.h"/>
    <Exclude path="P_dev_model/P_mes/filtring.c"/>
    <Exclude path="P_dev_model/P_mes/filtring.h"/>
    <Exclude path="P_dev_model/P_mes/mes_processing_sourse/processing_mesuremen_adc.c"/>
    <Exclude path="P_dev_model/P_mes/mes_processing_sourse/processing_mesuremen_adc.h"/>
    <Exclude path="P_dev_model/P_mes/mes_processing_sourse/processing_mesuremen_dma.c"/>
    <Exclude path="P_dev_model/P_mes/mes_processing_sourse/processing_mesuremen_dma.h"/>
    <Exclude path="P_dev_model/P_mes/processing_mesuremen_extern.h"/>
    <Exclude path="P_dev_model/P_mes/processing_mesuremen.c"/>
    <Exclude path="P_dev_model/P_mes/processing_mesuremen.h"/>
    <Exclude path="P_dev_model/P_TY_signal/1_processing_TY.c"/>
  </ExcludedFiles>
  <Drivers/>
</Project>
