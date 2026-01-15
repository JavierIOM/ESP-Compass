# Quick Hardware Setup Guide

## Parts List
- ESP32 Development Board
- Adafruit LSM303AGR (Product ID: 4413)
- STEMMA QT cable OR 4 jumper wires
- USB cable for programming

## Wiring Diagram

```
┌─────────────────┐
│   LSM303AGR     │
│  (Adafruit)     │
└─────────────────┘
      │ │ │ │
      │ │ │ └─── VIN  ──→  3.3V  ─┐
      │ │ └───── GND  ──→  GND   ─┤
      │ └─────── SDA  ──→  GPIO21 ─┤  ┌──────────┐
      └───────── SCL  ──→  GPIO22 ─┼──│  ESP32   │
                                    │  │   Dev    │
                           USB ─────┤  │  Board   │
                                    └──│          │
                                       └──────────┘
```

## Pin Connections

| LSM303AGR | ESP32 Pin | Notes |
|-----------|-----------|-------|
| VIN       | 3.3V      | Power supply |
| GND       | GND       | Ground |
| SDA       | GPIO 21   | I2C Data (default) |
| SCL       | GPIO 22   | I2C Clock (default) |

## Using STEMMA QT (Solderless)

If you have a STEMMA QT connector on your ESP32:
1. Plug the STEMMA QT cable into the LSM303AGR
2. Plug the other end into your ESP32's STEMMA QT connector
3. Done!

## Important Notes

- **Voltage:** Use 3.3V, not 5V (ESP32 GPIO pins are 3.3V)
- **I2C Addresses:**
  - Accelerometer: 0x19
  - Magnetometer: 0x1E
- **No external pull-up resistors needed** (built into the breakout board)

## Mounting Tips

- Mount the sensor flat and level for best results
- Keep away from magnetic interference (motors, speakers, large metal objects)
- Can be mounted in any orientation, but level is best for accuracy

## Testing the Hardware

After wiring, upload the sketch and open Serial Monitor (115200 baud):
- You should see "LSM303AGR sensors initialized successfully!"
- If you see an error, check your wiring
- The sensor LED should light up when powered

## Troubleshooting

**Problem:** "Could not find LSM303AGR" error

**Solutions:**
1. Check all wire connections
2. Verify 3.3V power is connected
3. Ensure SDA goes to GPIO 21, SCL to GPIO 22
4. Try a different USB cable (some are power-only)
5. Check if sensor LED lights up (indicates power is good)

**Problem:** Sensor found but readings are wrong

**Solutions:**
1. Keep sensor away from magnetic sources
2. Perform calibration (see README.md)
3. Ensure sensor is relatively level
