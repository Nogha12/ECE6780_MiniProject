/**
  ******************************************************************************
  * @file    i3g4250d.h
  * @author  Noah Lomu
  * @brief   This file contains definitions and function prototypes to be used
  *          with the MEMS motion sensor 3-axis digital output gyroscope.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I3G4250D_H
#define __I3G4250D_H

/************************  I3G4250D value definitions  ************************/
#define I3G4250D_WHO_AM_I_Value     (0xD3U)

/***********************  I3G4250D address definitions  ***********************/
#define I3G4250D_WHO_AM_I_Addr      (0x0FU)
#define I3G4250D_CTROL_REG1_Addr    (0x20U)
#define I3G4250D_CTROL_REG2_Addr    (0x21U)
#define I3G4250D_CTROL_REG3_Addr    (0x22U)
#define I3G4250D_CTROL_REG4_Addr    (0x23U)
#define I3G4250D_CTROL_REG5_Addr    (0x24U)
/* ... */
#define I3G4250D_OUT_X_L_Addr       (0x28U)
#define I3G4250D_OUT_X_H_Addr       (0x29U)
#define I3G4250D_OUT_Y_L_Addr       (0x2AU)
#define I3G4250D_OUT_Y_H_Addr       (0x2BU)
#define I3G4250D_OUT_Z_L_Addr       (0x2CU)
#define I3G4250D_OUT_Z_H_Addr       (0x2DU)
/* ... */

/*******************  Bit definition for SPI communication  *******************/
#define I3G4250D_SPI_MS_Pos         (6U)
#define I3G4250D_SPI_MS_Msk         (0x1U << I3G4250D_SPI_MS_Pos)                    /*!< 0x40 */
#define I3G4250D_SPI_MS             I3G4250D_SPI_MS_Msk                              /*!<Multiple byte mode enable */
#define I3G4250D_SPI_RW_Pos         (7U)
#define I3G4250D_SPI_RW_Msk         (0x1U << I3G4250D_SPI_RW_Pos)                    /*!< 0x80 */
#define I3G4250D_SPI_RW             I3G4250D_SPI_RW_Msk                              /*!<Read enable */

/*******************  Bit definition for CTRL_REG1 register  ******************/
#define I3G4250D_CTRL_REG1_Xen_Pos  (0U)
#define I3G4250D_CTRL_REG1_Xen_Msk  (0x1U << I3G4250D_CTRL_REG1_Xen_Pos)             /*!< 0x01 */
#define I3G4250D_CTRL_REG1_Xen      I3G4250D_CTRL_REG1_Xen_Msk                       /*!<X-axis enable */
#define I3G4250D_CTRL_REG1_Yen_Pos  (1U)
#define I3G4250D_CTRL_REG1_Yen_Msk  (0x1U << I3G4250D_CTRL_REG1_Yen_Pos)             /*!< 0x02 */
#define I3G4250D_CTRL_REG1_Yen      I3G4250D_CTRL_REG1_Yen_Msk                       /*!<Y-axis enable */
#define I3G4250D_CTRL_REG1_Zen_Pos  (2U)
#define I3G4250D_CTRL_REG1_Zen_Msk  (0x1U << I3G4250D_CTRL_REG1_Zen_Pos)             /*!< 0x04 */
#define I3G4250D_CTRL_REG1_Zen      I3G4250D_CTRL_REG1_Zen_Msk                       /*!<Z-axis enable */
#define I3G4250D_CTRL_REG1_PD_Pos   (3U)
#define I3G4250D_CTRL_REG1_PD_Msk   (0x1U << I3G4250D_CTRL_REG1_PD_Pos)              /*!< 0x08 */
#define I3G4250D_CTRL_REG1_PD       I3G4250D_CTRL_REG1_PD_Msk                        /*!<Power-down mode enable */

#define I3G4250D_CTRL_REG1_BW_Pos   (4U)
#define I3G4250D_CTRL_REG1_BW_Msk   (0x3U << I3G4250D_CTRL_REG1_BW_Pos)              /*!< 0x30 */
#define I3G4250D_CTRL_REG1_BW       I3G4250D_CTRL_REG1_BW_Msk                        /*!<BW1-BW0 (Bandwidth selection) */
#define I3G4250D_CTRL_REG1_BW_0     (0x1U << I3G4250D_CTRL_REG1_BW_Pos)              /*!< 0x10 */
#define I3G4250D_CTRL_REG1_BW_1     (0x2U << I3G4250D_CTRL_REG1_BW_Pos)              /*!< 0x20 */

#define I3G4250D_CTRL_REG1_DR_Pos   (6U)
#define I3G4250D_CTRL_REG1_DR_Msk   (0x3U << I3G4250D_CTRL_REG1_DR_Pos)              /*!< 0xC0 */
#define I3G4250D_CTRL_REG1_DR       I3G4250D_CTRL_REG1_DR_Msk                        /*!<DR1-DR0 (Output data rate selection) */
#define I3G4250D_CTRL_REG1_DR_0     (0x1U << I3G4250D_CTRL_REG1_DR_Pos)              /*!< 0x80 */
#define I3G4250D_CTRL_REG1_DR_1     (0x2U << I3G4250D_CTRL_REG1_DR_Pos)              /*!< 0x40 */

/*********************  Function prototypes (read/write)  *********************/
int I3G4250D_ReadRegister(uint8_t registerAddr, int bytesToRead);
void I3G4250D_WriteToRegister(uint8_t registerAddr, uint8_t data);

#endif /* __I3G4250D_H */