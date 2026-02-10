# STEPPER--Homing_angular
### Rutina de homing para 1 "Nema17" usando AccelStepper y 1 "hall effect sensor KY-035" (Base para rotot SCARA)
####  ALGORITMO:
  - Limita la búsqueda a ±90° mecánicos durante el homing
  - Detecta flancos de entrada y salida del imán
  - Calcula el centro geométrico del imán
  - Define ese centro como posición 0 (referencia absoluta)   
  - Usa velocidades rápidas y lentas para optimizar tiempo y precisión
  - Implementa un timeout y manejo de errores

####  HARDWARE:
  - MCU        : RP2040-Zero / (opcion para Arduino-Nano cambiando pins)
  - Driver     : Step/Dir compatible con AccelStepper
  - Botón      : Inicio de homing (con debounce)
  - LED        : Estado del homing
