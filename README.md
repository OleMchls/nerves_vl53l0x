# Nerves_VL53L0X

A library to interface with the VL53L0X laser time-of-flight sensor.

[![](https://cdn-shop.adafruit.com/970x728/3317-15.jpg)](https://www.adafruit.com/product/3317)

This is the 'big sister' of the VL6180X ToF sensor, and can handle about 50mm to 1200mm of range distance. If you need a smaller/closer range, check out the VL6180X which can measure 5mm to 200mm and also contains a light sensor.

## Installation

The package can be installed by adding `nerves_vl53l0x` to your list of dependencies in `mix.exs`:

```elixir
def deps do
  [
    {:nerves_vl53l0x, "~> 0.1.0"}
  ]
end
```

Documentation can be generated with [ExDoc](https://github.com/elixir-lang/ex_doc). Once published, the docs can be found at [https://hexdocs.pm/nerves_vl53l0x](https://hexdocs.pm/nerves_vl53l0x).

## Usage

```elixir
# Open the I2C bus, optionally you can also bas the device address
{:ok, ref} = VL53L0X.open("i2c-1")
# => {:ok, %VL53L0X{bus: #Reference<0.1403664192.269090823.21917>, device: 41, stop_variable: nil}}

# Initialize the sensor
{:ok, ref} = VL53L0X.init(ref)
# => {:ok, %VL53L0X{bus: #Reference<0.1403664192.269090823.21917>, device: 41, stop_variable: 0x29}}

# Optionally, the VL53L0X supports custom measuring time budgeting,
# the more time you allow the sensor for a measurement
# the more accurate the result will be.
# For example a higher speed but less accurate timing budget of 20ms:
VL53L0X.set_measurement_timing_budget(ref, 20_000)
# Or a slower but more accurate timing budget of 200ms:
VL53L0X.set_measurement_timing_budget(ref, 200_000)
# The default timing budget is 33ms, a good compromise of speed and accuracy.

# Read distance
distance_in_mm = VL53L0X.range(ref)
# => 147
```

## Acknowledgements

This library is basically a port of the [Adafruit python driver](https://github.com/adafruit/Adafruit_CircuitPython_VL53L0X) :heart: which in turn is based on the amazing work of [@pololu](https://github.com/pololu) on the arduino library [vl53l0x-arduino](https://github.com/pololu/vl53l0x-arduino).
