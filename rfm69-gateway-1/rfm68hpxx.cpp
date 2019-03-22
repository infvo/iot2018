void RFM69::setPASettings() {
  // disable OCP for high power devices, enable otherwise
  writeRegister(0x13, 0x0A | (_highPowerDevice ? 0x00 : 0x10));

if (highPowerDevice) {
      // enable PA1 only
      writeRegister(0x11, (readRegister(0x11) & 0x1F) | 0x40);
    } else {
      // enable PA0 only
      writeRegister(0x11, (readRegister(0x11) & 0x1F) | 0x80);
    }
}

// volgens HopeRF: reg 13 0x0F voor high power (PA1 and/or PA2), 0x1x voor Rx or PA0
// ik zou dan ipv. 0x0A verwachten: 0x0F???

// de tweede actie hierboven is het zetten van regPaLevel. Maar volgens mij zijn die een factor 2 mis? zie tabel 10... En waarom moet dat register eerst gelezen worden? Kun je niet gewoon de juiste settings invullen?

// ik mis: het zetten van regTestPa1 en regTestPa2????


//RadioHead code:

void RH_RF69::setTxPower(int8_t power, bool ishighpowermodule)
{
  _power = power;
  uint8_t palevel;
  
  if (ishighpowermodule)
  {
    if (_power < -2)
      _power = -2; //RFM69HW only works down to -2. 
    if (_power <= 13)
    {
      // -2dBm to +13dBm
      //Need PA1 exclusivelly on RFM69HW
      palevel = RH_RF69_PALEVEL_PA1ON | ((_power + 18) & 
      RH_RF69_PALEVEL_OUTPUTPOWER);
    }
    else if (_power >= 18)
    {
      // +18dBm to +20dBm
      // Need PA1+PA2
      // Also need PA boost settings change when tx is turned on and off, see setModeTx()
      palevel = RH_RF69_PALEVEL_PA1ON
	| RH_RF69_PALEVEL_PA2ON
	| ((_power + 11) & RH_RF69_PALEVEL_OUTPUTPOWER);
    }
    else
    {
      // +14dBm to +17dBm
      // Need PA1+PA2
      palevel = RH_RF69_PALEVEL_PA1ON
	| RH_RF69_PALEVEL_PA2ON
	| ((_power + 14) & RH_RF69_PALEVEL_OUTPUTPOWER);
    }
  }
  else
  {
    if (_power < -18) _power = -18;
    if (_power > 13) _power = 13; //limit for RFM69W
    palevel = RH_RF69_PALEVEL_PA0ON
      | ((_power + 18) & RH_RF69_PALEVEL_OUTPUTPOWER);
  }
  spiWrite(RH_RF69_REG_11_PALEVEL, palevel);
}
