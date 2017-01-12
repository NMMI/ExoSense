/*******************************************************************************
* File Name: BLUE_LED.h  
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

#if !defined(CY_PINS_BLUE_LED_H) /* Pins BLUE_LED_H */
#define CY_PINS_BLUE_LED_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "BLUE_LED_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    BLUE_LED_Write(uint8 value) ;
void    BLUE_LED_SetDriveMode(uint8 mode) ;
uint8   BLUE_LED_ReadDataReg(void) ;
uint8   BLUE_LED_Read(void) ;
uint8   BLUE_LED_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define BLUE_LED_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define BLUE_LED_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define BLUE_LED_DM_RES_UP          PIN_DM_RES_UP
#define BLUE_LED_DM_RES_DWN         PIN_DM_RES_DWN
#define BLUE_LED_DM_OD_LO           PIN_DM_OD_LO
#define BLUE_LED_DM_OD_HI           PIN_DM_OD_HI
#define BLUE_LED_DM_STRONG          PIN_DM_STRONG
#define BLUE_LED_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define BLUE_LED_MASK               BLUE_LED__MASK
#define BLUE_LED_SHIFT              BLUE_LED__SHIFT
#define BLUE_LED_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define BLUE_LED_PS                     (* (reg8 *) BLUE_LED__PS)
/* Data Register */
#define BLUE_LED_DR                     (* (reg8 *) BLUE_LED__DR)
/* Port Number */
#define BLUE_LED_PRT_NUM                (* (reg8 *) BLUE_LED__PRT) 
/* Connect to Analog Globals */                                                  
#define BLUE_LED_AG                     (* (reg8 *) BLUE_LED__AG)                       
/* Analog MUX bux enable */
#define BLUE_LED_AMUX                   (* (reg8 *) BLUE_LED__AMUX) 
/* Bidirectional Enable */                                                        
#define BLUE_LED_BIE                    (* (reg8 *) BLUE_LED__BIE)
/* Bit-mask for Aliased Register Access */
#define BLUE_LED_BIT_MASK               (* (reg8 *) BLUE_LED__BIT_MASK)
/* Bypass Enable */
#define BLUE_LED_BYP                    (* (reg8 *) BLUE_LED__BYP)
/* Port wide control signals */                                                   
#define BLUE_LED_CTL                    (* (reg8 *) BLUE_LED__CTL)
/* Drive Modes */
#define BLUE_LED_DM0                    (* (reg8 *) BLUE_LED__DM0) 
#define BLUE_LED_DM1                    (* (reg8 *) BLUE_LED__DM1)
#define BLUE_LED_DM2                    (* (reg8 *) BLUE_LED__DM2) 
/* Input Buffer Disable Override */
#define BLUE_LED_INP_DIS                (* (reg8 *) BLUE_LED__INP_DIS)
/* LCD Common or Segment Drive */
#define BLUE_LED_LCD_COM_SEG            (* (reg8 *) BLUE_LED__LCD_COM_SEG)
/* Enable Segment LCD */
#define BLUE_LED_LCD_EN                 (* (reg8 *) BLUE_LED__LCD_EN)
/* Slew Rate Control */
#define BLUE_LED_SLW                    (* (reg8 *) BLUE_LED__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define BLUE_LED_PRTDSI__CAPS_SEL       (* (reg8 *) BLUE_LED__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define BLUE_LED_PRTDSI__DBL_SYNC_IN    (* (reg8 *) BLUE_LED__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define BLUE_LED_PRTDSI__OE_SEL0        (* (reg8 *) BLUE_LED__PRTDSI__OE_SEL0) 
#define BLUE_LED_PRTDSI__OE_SEL1        (* (reg8 *) BLUE_LED__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define BLUE_LED_PRTDSI__OUT_SEL0       (* (reg8 *) BLUE_LED__PRTDSI__OUT_SEL0) 
#define BLUE_LED_PRTDSI__OUT_SEL1       (* (reg8 *) BLUE_LED__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define BLUE_LED_PRTDSI__SYNC_OUT       (* (reg8 *) BLUE_LED__PRTDSI__SYNC_OUT) 


#if defined(BLUE_LED__INTSTAT)  /* Interrupt Registers */

    #define BLUE_LED_INTSTAT                (* (reg8 *) BLUE_LED__INTSTAT)
    #define BLUE_LED_SNAP                   (* (reg8 *) BLUE_LED__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins BLUE_LED_H */


/* [] END OF FILE */
