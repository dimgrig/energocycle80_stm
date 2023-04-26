#include "pwr.h"

void pwr_init() {

	/* Disable the PVD Output */
	HAL_PWR_DisablePVD();

	/* Configure the PVD Level to */
    PWR_PVDTypeDef sConfigPVD;
    sConfigPVD.PVDLevel = PWR_PVDLEVEL_7;
    sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING;
    HAL_PWR_ConfigPVD(&sConfigPVD);

	/* Enable the PVD Output */
	HAL_PWR_EnablePVD();
}


