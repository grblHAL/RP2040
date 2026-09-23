/*
 * Mapeo de hardware personalizado para Raspberry Pi Pico (CNC)
 * Configuración con pines de Habilitación (Enable) separados
 * para permitir movimiento manual de los ejes en el taller.
 */

// 1. Pines de Paso (PUL / STEP)
// Lógica: Asignamos la base en 2. El sistema PIO de la Pico asigna 
// automáticamente y de forma consecutiva: X=2, Y=3, Z=4.
#define STEP_PORT           GPIO_PIO  
#define STEP_PINS_BASE      2         

// 2. Pines de Dirección (DIR)
// Lógica: Define el sentido de giro de cada eje.
#define DIRECTION_PORT      GPIO_OUTPUT
#define X_DIRECTION_PIN     5
#define Y_DIRECTION_PIN     6
#define Z_DIRECTION_PIN     7
#define DIRECTION_OUTMODE   GPIO_SHIFT5

// 3. Pines de Habilitación (ENA) Independientes
// Lógica: Reemplaza el pin global para controlar el torque de cada 
// motor de forma individual y evitar conflictos en los convertidores.
#define ENABLE_PORT         GPIO_OUTPUT
#define X_ENABLE_PIN        8
#define Y_ENABLE_PIN        9
#define Z_ENABLE_PIN        10

// 4. Finales de Carrera (Límites) - COMENTADOS
// Lógica: Los desactivamos anulando el código original para que no 
// interfieran con los nuevos pines de Enable asignados al 9 y 10.
// #define X_LIMIT_PIN         9
// #define Y_LIMIT_PIN         10
// #define Z_LIMIT_PIN         11
// #define LIMIT_INMODE        GPIO_MAP

// 5. Pin de Sonda (Touch Probe)
// Lógica: Entrada auxiliar para detectar el cero del eje Z al hacer 
// contacto con GND a través de la herramienta.
#define AUXINPUT2_PIN       28 
#define PROBE_PIN           AUXINPUT2_PIN

// ==============================================================
// 6. PINES DE CONTROL DE SISTEMA (CORRECCIÓN ERROR 79 Y FEEDHOLD)
// ==============================================================
// Lógica: Comentamos explícitamente las definiciones de control 
// para evitar que lean ruido eléctrico al estar flotando.

// Anula la búsqueda física del Paro de Emergencia (E-Stop) en el GPIO 22.
// #define RESET_PIN           22 

// Anula la Pausa Física. Libera el GPIO 7 para que el Eje Z funcione.
// #define FEED_HOLD_PIN       7  

// Anula el botón de Inicio de Ciclo físico para evitar conflictos.
// #define CYCLE_START_PIN     8   ,ok que hago con este código , con este codigo corregimos el problema del ruido
