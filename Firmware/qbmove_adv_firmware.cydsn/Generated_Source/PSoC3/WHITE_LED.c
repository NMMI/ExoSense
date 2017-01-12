/*******************************************************************************
* File Name: WHITE_LED.c  
* Version 2.10
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "WHITE_LED.h"


/*******************************************************************************
* Function Name: WHITE_LED_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None 
*  
*******************************************************************************/
void WHITE_LED_Write(uint8 value) 
{
    uint8 staticBits = (WHITE_LED_DR & (uint8)(~WHITE_LED_MASK));
    WHITE_LED_DR = staticBits | ((uint8)(value << WHITE_LED_SHIFT) & WHITE_LED_MASK);
}


/*******************************************************************************
* Function Name: WHITE_LED_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  WHITE_LED_DM_STRONG     Strong Drive 
*  WHITE_LED_DM_OD_HI      Open Drain, Drives High 
*  WHITE_LED_DM_OD_LO      Open Drain, Drives Low 
*  WHITE_LED_DM_RES_UP     Resistive Pull Up 
*  WHITE_LED_DM_RES_DWN    Resistive Pull Down 
*  WHITE_LED_DM_RES_UPDWN  Resistive Pull Up/Down 
*  WHITE_LED_DM_DIG_HIZ    High Impedance Digital 
*  WHITE_LED_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void WHITE_LED_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(WHITE_LED_0, mode);
}


/*******************************************************************************
* Function Name: WHITE_LED_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro WHITE_LED_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 WHITE_LED_Read(void) 
{
    return (WHITE_LED_PS & WHITE_LED_MASK) >> WHITE_LED_SHIFT;
}


/*******************************************************************************
* Function Name: WHITE_LED_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 WHITE_LED_ReadDataReg(void) 
{
    return (WHITE_LED_DR & WHITE_LED_MASK) >> WHITE_LED_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(WHITE_LED_INTSTAT) 

    /*******************************************************************************
    * Function Name: WHITE_LED_ClearInterrupt
    ********************************************************************************
    *
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 WHITE_LED_ClearInterrupt(void) 
    {
        return (WHITE_LED_INTSTAT & WHITE_LED_MASK) >> WHITE_LED_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
