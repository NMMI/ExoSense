/*******************************************************************************
* File Name: WHITE_LED.h  
* Version 2.10
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_WHITE_LED_H) /* Pins WHITE_LED_H */
#define CY_PINS_WHITE_LED_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "WHITE_LED_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    WHITE_LED_Write(uint8 value) ;
void    WHITE_LED_SetDriveMode(uint8 mode) ;
uint8   WHITE_LED_ReadDataReg(void) ;
uint8   WHITE_LED_Read(void) ;
uint8   WHITE_LED_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define WHITE_LED_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define WHITE_LED_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define WHITE_LED_DM_RES_UP          PIN_DM_RES_UP
#define WHITE_LED_DM_RES_DWN         PIN_DM_RES_DWN
#define WHITE_LED_DM_OD_LO           PIN_DM_OD_LO
#define WHITE_LED_DM_OD_HI           PIN_DM_OD_HI
#define WHITE_LED_DM_STRONG          PIN_DM_STRONG
#define WHITE_LED_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define WHITE_LED_MASK               WHITE_LED__MASK
#define WHITE_LED_SHIFT              WHITE_LED__SHIFT
#define WHITE_LED_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define WHITE_LED_PS                     (* (reg8 *) WHITE_LED__PS)
/* Data Register */
#define WHITE_LED_DR                     (* (reg8 *) WHITE_LED__DR)
/* Port Number */
#define WHITE_LED_PRT_NUM                (* (reg8 *) WHITE_LED__PRT) 
/* Connect to Analog Globals */                                                  
#define WHITE_LED_AG                     (* (reg8 *) WHITE_LED__AG)                       
/* Analog MUX bux enable */
#define WHITE_LED_AMUX                   (* (reg8 *) WHITE_LED__AMUX) 
/* Bidirectional Enable */                                                        
#define WHITE_LED_BIE                    (* (reg8 *) WHITE_LED__BIE)
/* Bit-mask for Aliased Register Access */
#define WHITE_LED_BIT_MASK               (* (reg8 *) WHITE_LED__BIT_MASK)
/* Bypass Enable */
#define WHITE_LED_BYP                    (* (reg8 *) WHITE_LED__BYP)
/* Port wide control signals */                                                   
#define WHITE_LED_CTL                    (* (reg8 *) WHITE_LED__CTL)
/* Drive Modes */
#define WHITE_LED_DM0                    (* (reg8 *) WHITE_LED__DM0) 
#define WHITE_LED_DM1                    (* (reg8 *) WHITE_LED__DM1)
#define WHITE_LED_DM2                    (* (reg8 *) WHITE_LED__DM2) 
/* Input Buffer Disable Override */
#define WHITE_LED_INP_DIS                (* (reg8 *) WHITE_LED__INP_DIS)
/* LCD Common or Segment Drive */
#define WHITE_LED_LCD_COM_SEG            (* (reg8 *) WHITE_LED__LCD_COM_SEG)
/* Enable Segment LCD */
#define WHITE_LED_LCD_EN                 (* (reg8 *) WHITE_LED__LCD_EN)
/* Slew Rate Control */
#define WHITE_LED_SLW                    (* (reg8 *) WHITE_LED__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define WHITE_LED_PRTDSI__CAPS_SEL       (* (reg8 *) WHITE_LED__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define WHITE_LED_PRTDSI__DBL_SYNC_IN    (* (reg8 *) WHITE_LED__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define WHITE_LED_PRTDSI__OE_SEL0        (* (reg8 *) WHITE_LED__PRTDSI__OE_SEL0) 
#define WHITE_LED_PRTDSI__OE_SEL1        (* (reg8 *) WHITE_LED__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define WHITE_LED_PRTDSI__OUT_SEL0       (* (reg8 *) WHITE_LED__PRTDSI__OUT_SEL0) 
#define WHITE_LED_PRTDSI__OUT_SEL1       (* (reg8 *) WHITE_LED__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define WHITE_LED_PRTDSI__SYNC_OUT       (* (reg8 *) WHITE_LED__PRTDSI__SYNC_OUT) 


#if defined(WHITE_LED__INTSTAT)  /* Interrupt Registers */

    #define WHITE_LED_INTSTAT                (* (reg8 *) WHITE_LED__INTSTAT)
    #define WHITE_LED_SNAP                   (* (reg8 *) WHITE_LED__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins WHITE_LED_H */


/* [] END OF FILE */
