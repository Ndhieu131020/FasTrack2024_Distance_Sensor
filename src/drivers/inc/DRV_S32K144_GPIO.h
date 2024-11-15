#ifndef DRV_S32K144_GPIO_H_
#define DRV_S32K144_GPIO_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Enum: GPIO Pin Direction*/
typedef enum
{
    Input_Direction  = 0U,
    Output_Direction = 1U,
} GPIO_Data_Direction_t;

/* Enum: GPIO Pin Data Output */
typedef enum
{
    GPIO_PIN_0_LOGIC = 0U,
    GPIO_PIN_1_LOGIC = 1U,
} GPIO_Data_Output_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/

/*##############################################################################
**
* @brief: Set Pins Direction (Input or Output)
*
* @param [in] Port_Ins: Port index corresponding to the port base (PORTA, PORTB, PORTC, etc.)
* @param [in] Pin: Pin number of the port
* @param [in] mode: GPIO mode (Input or Output)
* @return
*
*#############################################################################*/
void DRV_GPIO_SetPinDirection(const uint8_t               Port_Ins,
                              const uint8_t               Pin,
                              const GPIO_Data_Direction_t mode);

/*##############################################################################
**
* @brief: Read Input Pins
*
* @param [in] Port_Ins: Port index corresponding to the port base (PORTA, PORTB, PORTC, etc.)
* @param [in] Pin: Pin number of the port
* @return: Status of pin input
*
*#############################################################################*/
uint32_t DRV_GPIO_ReadPins(const uint8_t Port_Ins, const uint8_t Pin);

/*##############################################################################
**
* @brief: Write Output Pins
*
* @param [in] Port_Ins: Port index corresponding to the port base (PORTA, PORTB, PORTC, etc.)
* @param [in] Pin: Pin number of the port
* @param [in] mode: GPIO data output (logic 0 or logic 1)
* @return
*
*#############################################################################*/
void DRV_GPIO_WritePins(const uint8_t            Port_Ins,
                        const uint8_t            Pin,
                        const GPIO_Data_Output_t mode);

/*##############################################################################
**
* @brief: Set Output Pins
*
* @param [in] Port_Ins: Port index corresponding to the port base (PORTA, PORTB, PORTC, etc.)
* @param [in] Pin: Pin number of the port
* @return
*
*#############################################################################*/
void DRV_GPIO_SetPins(const uint8_t Port_Ins, const uint8_t Pin);

/*##############################################################################
**
* @brief: Clear Output Pins
*
* @param [in] Port_Ins: Port index corresponding to the port base (PORTA, PORTB, PORTC, etc.)
* @param [in] Pin: Pin number of the port
* @return
*
*#############################################################################*/
void DRV_GPIO_ClearPins(const uint8_t Port_Ins, const uint8_t Pin);

/*##############################################################################
**
* @brief: Toggle Output Pins
*
* @param [in] Port_Ins: Port index corresponding to the port base (PORTA, PORTB, PORTC, etc.)
* @param [in] Pin: Pin number of the port
* @return
*
*#############################################################################*/
void DRV_GPIO_TogglePins(const uint8_t Port_Ins, const uint8_t Pin);

#endif /* DRV_S32K144_GPIO_H_ */

/*******************************************************************************
 * End Of File
 ******************************************************************************/

