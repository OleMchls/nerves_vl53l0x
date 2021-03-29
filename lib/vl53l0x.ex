defmodule VL53L0X do
  @moduledoc """
  A library to interface with the VL6180X time-of-flight laser sensor.
  """
  use Bitwise

  alias Circuits.I2C

  @type t() :: %VL53L0X{bus: I2C.bus(), device: I2C.address(), stop_variable: 0x00..0xFF}
  defstruct [:bus, :device, :stop_variable]

  @vl53l0x_default_i2c_addr 0x29

  # Configuration constants:
  @sysrange_start 0x00
  @system_thresh_high 0x0C
  @system_thresh_low 0x0E
  @system_sequence_config 0x01
  @system_range_config 0x09
  @system_intermeasurement_period 0x04
  @system_interrupt_config_gpio 0x0A
  @gpio_hv_mux_active_high 0x84
  @system_interrupt_clear 0x0B
  @result_interrupt_status 0x13
  @result_range_status 0x14
  @result_core_ambient_window_events_rtn 0xBC
  @result_core_ranging_total_events_rtn 0xC0
  @result_core_ambient_window_events_ref 0xD0
  @result_core_ranging_total_events_ref 0xD4
  @result_peak_signal_rate_ref 0xB6
  @algo_part_to_part_range_offset_mm 0x28
  @i2c_slave_device_address 0x8A
  @msrc_config_control 0x60
  @pre_range_config_min_snr 0x27
  @pre_range_config_valid_phase_low 0x56
  @pre_range_config_valid_phase_high 0x57
  @pre_range_min_count_rate_rtn_limit 0x64
  @final_range_config_min_snr 0x67
  @final_range_config_valid_phase_low 0x47
  @final_range_config_valid_phase_high 0x48
  @final_range_config_min_count_rate_rtn_limit 0x44
  @pre_range_config_sigma_thresh_hi 0x61
  @pre_range_config_sigma_thresh_lo 0x62
  @pre_range_config_vcsel_period 0x50
  @pre_range_config_timeout_macrop_hi 0x51
  @pre_range_config_timeout_macrop_lo 0x52
  @system_histogram_bin 0x81
  @histogram_config_initial_phase_select 0x33
  @histogram_config_readout_ctrl 0x55
  @final_range_config_vcsel_period 0x70
  @final_range_config_timeout_macrop_hi 0x71
  @final_range_config_timeout_macrop_lo 0x72
  @crosstalk_compensation_peak_rate_mcps 0x20
  @msrc_config_timeout_macrop 0x46
  @soft_reset_go2_soft_reset_n 0xBF
  @identification_model_id 0xC0
  @identification_revision_id 0xC2
  @osc_calibrate_val 0xF8
  @global_config_vcsel_width 0x32
  @global_config_spad_enables_ref_0 0xB0
  @global_config_spad_enables_ref_1 0xB1
  @global_config_spad_enables_ref_2 0xB2
  @global_config_spad_enables_ref_3 0xB3
  @global_config_spad_enables_ref_4 0xB4
  @global_config_spad_enables_ref_5 0xB5
  @global_config_ref_en_start_select 0xB6
  @dynamic_spad_num_requested_ref_spad 0x4E
  @dynamic_spad_ref_en_start_offset 0x4F
  @power_management_go1_power_force 0x80
  @vhv_config_pad_scl_sda__extsup_hv 0x89
  @algo_phasecal_lim 0x30
  @algo_phasecal_config_timeout 0x30
  @vcsel_period_pre_range 0
  @vcsel_period_final_range 1

  def open(bus_name, address \\ @vl53l0x_default_i2c_addr) do
    {:ok, ref} = I2C.open(bus_name)

    # Check check_device_model_identification
    {:ok, <<0xEE>>} = readu8(ref, address, @identification_model_id)

    {:ok, %__MODULE__{bus: ref, device: address}}
  end

  @spec init(t()) :: {:ok, t()} | {:error, term()}
  def init(%__MODULE__{bus: bus, device: device} = sensor) do
    # Initialize access to the sensor.  This is based on the logic from:
    #   https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
    # Set I2C standard mode.
    for {register, val} <- [{0x88, 0x00}, {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00}],
        do: writeu8(bus, device, register, val)

    {:ok, <<stop_variable>>} = readu8(bus, device, 0x91)
    sensor = %{sensor | stop_variable: stop_variable}

    for {register, val} <- [{0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}],
        do: writeu8(bus, device, register, val)

    # disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4)
    # limit checks
    {:ok, <<config_control>>} = readu8(bus, device, @msrc_config_control)
    writeu8(bus, device, @msrc_config_control, config_control ||| 0b0001_0010)

    # set final range signal rate limit to 0.25 MCPS (million counts per second)
    set_signal_rate_limit(sensor, 0.25)
    writeu8(bus, device, @system_sequence_config, 0xFF)
    # NEXT: Adapt get_spad_info
    {spad_count, spad_is_aperture} =
      get_spad_info(sensor)
      |> IO.inspect(label: "SPAD INFO")

    {:ok, ref_spad} = read_multi(bus, device, @global_config_spad_enables_ref_0, 6)

    ref_spad_map =
      ref_spad
      |> IO.inspect(label: "ref_spad")
      |> :binary.bin_to_list()
      # Insert index as value
      |> Enum.with_index()
      # Swap value and key
      |> Enum.into(%{}, fn {v, k} -> {k, v} end)
      |> IO.inspect(label: "ref_spad_map")

    for {register, val} <- [
          {0xFF, 0x01},
          {@dynamic_spad_ref_en_start_offset, 0x00},
          {@dynamic_spad_num_requested_ref_spad, 0x2C},
          {0xFF, 0x00},
          {@global_config_ref_en_start_select, 0xB4}
        ],
        do: writeu8(bus, device, register, val)

    # 12 is the first aperture spad
    first_spad_to_enable = if spad_is_aperture, do: 12, else: 0
    spads_enabled = 0

    {_spads_enabled, ref_spad_map} =
      Enum.reduce(0..47, {spads_enabled, ref_spad_map}, fn i,
                                                           {inner_spads_enabled,
                                                            inner_ref_spad_map} ->
        row = div(i, 8)

        if i < first_spad_to_enable || inner_spads_enabled == spad_count do
          # This bit is lower than the first one that should be enabled, or
          # (reference_spad_count) bits have already been enabled, so zero this bit
          {inner_spads_enabled,
           Map.update!(inner_ref_spad_map, row, fn spads -> spads &&& ~~~(1 <<< rem(i, 8)) end)}
          |> IO.inspect(label: "#{i}: spads action first")
        else
          if inner_ref_spad_map[row] >>> rem(i, 8) &&& 0x1 do
            {inner_spads_enabled + 1, inner_ref_spad_map}
            |> IO.inspect(label: "#{i}: spads action second")
          else
            {inner_spads_enabled, inner_ref_spad_map}
            |> IO.inspect(label: "#{i}: spads action none")
          end
        end
      end)
      |> IO.inspect(label: "spads action")

    spad_regs =
      ref_spad_map
      |> Map.values()
      |> :binary.list_to_bin()
      |> IO.inspect(label: "spads reg")

    write_multi(bus, device, @global_config_spad_enables_ref_0, spad_regs)

    # -- VL53L0X_load_tuning_settings()
    for {register, val} <- [
          {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00},
          {0x10, 0x00}, {0x11, 0x00}, {0x24, 0x01}, {0x25, 0xFF},
          {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
          {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00},
          {0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25},
          {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
          {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00},
          {0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},
          {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
          {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00},
          {0x78, 0x21}, {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00},
          {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
          {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00},
          {0x34, 0x03}, {0x35, 0x44}, {0xFF, 0x01}, {0x31, 0x04},
          {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
          {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28},
          {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE},
          {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
          {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01},
          {0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}
        ],
        do: writeu8(bus, device, register, val)

    # Set interrupt config to new sample ready"
    writeu8(bus, device, @system_interrupt_config_gpio, 0x04)
    {:ok, << gpio_hv_mux >>} = readu8(bus, device, @gpio_hv_mux_active_high)
    writeu8(bus, device, @gpio_hv_mux_active_high, gpio_hv_mux &&& ~~~0x10) # active low
    writeu8(bus, device, @system_interrupt_clear, 0x01)

    measurement_timing_budget_us = get_measurement_timing_budget(sensor)
    |> IO.inspect(label: "measurement_timing_budget")
    writeu8(bus, device, @system_sequence_config, 0xE8)
    set_measurement_timing_budget(sensor, measurement_timing_budget_us)

    writeu8(bus, device, @system_sequence_config, 0x01)
    perform_single_ref_calibration(sensor, 0x40)
    writeu8(bus, device, @system_sequence_config, 0x02)
    perform_single_ref_calibration(sensor, 0x00)
    # "restore the previous Sequence Config"
    writeu8(bus, device, @system_sequence_config, 0xE8)

    {:ok, sensor}
  end

  def set_signal_rate_limit(%__MODULE__{}, limit_mcps) when limit_mcps < 0,
    do: {:error, :limit_mcps_too_low}

  def set_signal_rate_limit(%__MODULE__{}, limit_mcps) when limit_mcps > 511.99,
    do: {:error, :limit_mcps_too_high}

  def set_signal_rate_limit(%__MODULE__{bus: bus, device: device}, limit_mcps) do
    writeu16(
      bus,
      device,
      @final_range_config_min_count_rate_rtn_limit,
      trunc(limit_mcps * (1 <<< 7))
    )
  end

  def get_spad_info(%__MODULE__{bus: bus, device: device}) do
    # Get reference SPAD count and type, returned as a 2-tuple of
    # count and boolean is_aperture.  Based on code from:
    #   https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
    for {register, val} <- [{0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x06}],
        do: writeu8(bus, device, register, val)

    {:ok, <<reg_eightthree>>} = readu8(bus, device, 0x83)
    writeu8(bus, device, 0x83, reg_eightthree ||| 0x04)

    for {register, val} <- [{0xFF, 0x07}, {0x81, 0x01}, {0x80, 0x01}, {0x94, 0x6B}, {0x83, 0x00}],
        do: writeu8(bus, device, register, val)

    readu8_until(bus, device, 0x83, fn
      <<0x00>> -> false
      _ -> true
    end)

    writeu8(bus, device, 0x83, 0x01)
    {:ok, <<is_aperture::1, count::7>>} = readu8(bus, device, 0x83)

    for {register, val} <- [{0x81, 0x00}, {0xFF, 0x06}], do: writeu8(bus, device, register, val)
    {:ok, <<reg_eightthree>>} = readu8(bus, device, 0x83)
    writeu8(bus, device, 0x83, reg_eightthree &&& ~~~0x04)

    for {register, val} <- [{0xFF, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}],
        do: writeu8(bus, device, register, val)

    {count, is_aperture == 1}
  end

  def perform_single_ref_calibration(%__MODULE__{bus: bus, device: device}, vhv_init_byte) do
    # based on VL53L0X_perform_single_ref_calibration() from ST API.
    writeu8(bus, device, @sysrange_start, 0x01 ||| vhv_init_byte &&& 0xFF)
    readu8_until(bus, device, @result_interrupt_status, fn
      << _::5, 0::3 >> -> false
      _ -> true
    end)
    writeu8(bus, device, @system_interrupt_clear, 0x01)
    writeu8(bus, device, @sysrange_start, 0x00)
  end

  def get_measurement_timing_budget(%__MODULE__{} = ref) do
    {tcc, dss, msrc, pre_range, final_range} = get_sequence_step_enables(ref)
    |> IO.inspect(label: "get_measurement_timing_budget:get_sequence_step_enables")
    {msrc_dss_tcc_us, pre_range_us, final_range_us, _, _} = get_sequence_step_timeouts(ref, pre_range)
    |> IO.inspect(label: "get_measurement_timing_budget:get_sequence_step_timeouts")

    budget_us = 1910 + 960  # Start overhead + end overhead.
    budget_us = if tcc, do: budget_us + msrc_dss_tcc_us + 590, else: budget_us
    budget_us = if dss do
      budget_us + msrc_dss_tcc_us + 590
     else
       if msrc, do: budget_us + msrc_dss_tcc_us + 660, else: budget_us
     end
     budget_us = if pre_range, do: budget_us + pre_range_us + 660, else: budget_us
     budget_us = if final_range, do: budget_us + final_range_us + 550, else: budget_us

    budget_us
  end

  def set_measurement_timing_budget(%__MODULE__{}, budget_us) when budget_us < 20000, do: {:error, :budget_too_small}
  def set_measurement_timing_budget(%__MODULE__{bus: bus, device: device} = ref, budget_us) when budget_us >= 20000  do
    {tcc, dss, msrc, pre_range, final_range} = get_sequence_step_enables(ref)
    |> IO.inspect(label: "set_measurement_timing_budget:get_sequence_step_enables")
    {msrc_dss_tcc_us, pre_range_us, _, final_range_vcsel_period_pclks, pre_range_mclks} = get_sequence_step_timeouts(ref, pre_range)
    |> IO.inspect(label: "set_measurement_timing_budget:get_sequence_step_timeouts")

    used_budget_us = 1320 + 960  # Start (diff from get) + end overhead
    used_budget_us = if tcc, do: used_budget_us + msrc_dss_tcc_us + 590, else: used_budget_us
    used_budget_us = if dss do
      used_budget_us + 2 * (msrc_dss_tcc_us + 690)
     else
       if msrc, do: used_budget_us + msrc_dss_tcc_us + 660, else: used_budget_us
     end
     used_budget_us = if pre_range, do: used_budget_us + pre_range_us + 660, else: used_budget_us
     if final_range do
       IO.inspect(used_budget_us, label: "final used_budget_us")
       used_budget_us = used_budget_us + 550
       if used_budget_us > budget_us, do: throw({:error, :budget_too_big})
       final_range_timeout_us = budget_us - used_budget_us
       final_range_timeout_mclks = timeout_microseconds_to_mclks(final_range_timeout_us, final_range_vcsel_period_pclks)
       final_range_timeout_mclks = if pre_range, do: final_range_timeout_mclks + pre_range_mclks, else: final_range_timeout_mclks
       writeu16(bus, device, @final_range_config_timeout_macrop_hi, encode_timeout(final_range_timeout_mclks))
     end
  end

  def range(%__MODULE__{stop_variable: nil}), do: {:error, :not_initialized}
  def range(%__MODULE__{bus: bus, device: device, stop_variable: stop_variable}) do
    for {register, val} <- [
            {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00}, {0x91, stop_variable},
            {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}, {@sysrange_start, 0x01}
      ],
        do: writeu8(bus, device, register, val)

    readu8_until(bus, device, @sysrange_start, fn
      << _::7, 0::1 >> -> true
      _ -> false
    end)
    readu8_until(bus, device, @result_interrupt_status, fn
      << _::5, 0::3 >> -> false
      _ -> true
    end)

    # assumptions: Linearity Corrective Gain is 1000 (default)
    # fractional ranging is not enabled
    range_mm = read_multi(bus, device, @result_range_status + 10, 2) |> unpack_16
    writeu8(bus, device, @system_interrupt_clear, 0x01)
    range_mm
  end

  defp get_sequence_step_enables(%__MODULE__{bus: bus, device: device}) do
    {:ok, << final_range::1, pre_range::1, _::1, tcc::1, dss::1, msrc::1, _::2 >>} = readu8(bus, device, @system_sequence_config)

    {tcc == 1, dss == 1, msrc == 1, pre_range == 1, final_range == 1}
  end

  # based on get_sequence_step_timeout() from ST API but modified by
  # pololu here:
  #   https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
  defp get_sequence_step_timeouts(%__MODULE__{bus: bus, device: device} = ref, pre_range) do
    pre_range_vcsel_period_pclks = get_vcsel_pulse_period(ref, @vcsel_period_pre_range)

    msrc_dss_tcc_mclks = (read_reg(ref, @msrc_config_timeout_macrop) + 1) &&& 0xFF
    msrc_dss_tcc_us = timeout_mclks_to_microseconds(msrc_dss_tcc_mclks, pre_range_vcsel_period_pclks)

    pre_range_mclks = read_multi(bus, device, @pre_range_config_timeout_macrop_hi, 2) |> unpack_16 |> decode_timeout()
    pre_range_us = timeout_mclks_to_microseconds(pre_range_mclks, pre_range_vcsel_period_pclks)

    final_range_vcsel_period_pclks = get_vcsel_pulse_period(ref, @vcsel_period_final_range)
    final_range_mclks = read_multi(bus, device, @final_range_config_timeout_macrop_hi, 2) |> unpack_16 |> decode_timeout()
    final_range_mclks = if pre_range, do: final_range_mclks - pre_range_mclks, else: final_range_mclks
    final_range_us = timeout_mclks_to_microseconds(final_range_mclks, final_range_vcsel_period_pclks)

    {msrc_dss_tcc_us, pre_range_us, final_range_us, final_range_vcsel_period_pclks, pre_range_mclks}
  end

  defp get_vcsel_pulse_period(ref, @vcsel_period_pre_range) do
    ref
    |> read_reg(@pre_range_config_vcsel_period)
    |> decode_vcsel_period()
  end
  defp get_vcsel_pulse_period(ref, @vcsel_period_final_range) do
    ref
    |> read_reg(@final_range_config_vcsel_period)
    |> decode_vcsel_period()
  end
  defp get_vcsel_pulse_period(%__MODULE__{}, _), do: 255
  defp decode_vcsel_period(reg_val), do: (((reg_val) + 1) <<< 1)

  defp timeout_mclks_to_microseconds(timeout_period_mclks, vcsel_period_pclks) do
    macro_period_ns = div((2304 * (vcsel_period_pclks) * 1655) + 500, 1000)
    div((timeout_period_mclks * macro_period_ns) + div(macro_period_ns, 2), 1000)
  end

  defp timeout_microseconds_to_mclks(timeout_period_us, vcsel_period_pclks) do
    macro_period_ns = div((2304 * (vcsel_period_pclks) * 1655) + 500, 1000)
    div((timeout_period_us * 1000) + div(macro_period_ns, 2), macro_period_ns)
  end

  defp decode_timeout(val) do
    # format: "(LSByte * 2^MSByte) + 1"
    floor(((val &&& 0xFF) / 1) * :math.pow(2.0, ((val &&& 0xFF00) >>> 8)) + 1)
  end

  defp encode_timeout(timeout_mclks) when timeout_mclks <= 0, do: 0
  defp encode_timeout(timeout_mclks) when timeout_mclks > 0xFFFF, do: encode_timeout(0xFFFF)
  defp encode_timeout(timeout_mclks) do
    # format: "(LSByte * 2^MSByte) + 1"
    {ls_byte, ms_byte} = split_bytes(timeout_mclks - 1, 0)
    ((ms_byte <<< 8) ||| (ls_byte &&& 0xFF)) &&& 0xFFFF
  end

  defp split_bytes(ls_byte, ms_byte) when ls_byte > 255, do: split_bytes(ls_byte >>> 1, ms_byte + 1)
  defp split_bytes(ls_byte, ms_byte), do: {ls_byte, ms_byte}

  defp writeu8(bus, device, address, data) do
    I2C.write(bus, device, <<address &&& 0xFF::8, data &&& 0xFF>>)
  end

  defp writeu16(bus, device, address, data) do
    write_multi(bus, device, address, << data::16 >>)
  end

  def write_multi(bus, device, address, data) when is_binary(data) do
    I2C.write(bus, device, << address &&& 0xFF::8 >> <> data)
  end

  defp read_reg(%__MODULE__{bus: bus, device: device}, address) do
    {:ok, << val >>} = readu8(bus, device, address)
    val
  end

  def readu8(bus, device, address) do
    read_multi(bus, device, address, 1)
  end

  def read_multi(bus, device, address, num_reg) do
    I2C.write_read(bus, device, <<address &&& 0xFF>>, num_reg)
  end

  defp readu8_until(bus, device, address, until) do
    {:ok, result} = readu8(bus, device, address)

    if until.(result), do: result, else: readu8_until(bus, device, address, until)
  end

  defp unpack_16({:ok, << val::16 >>}), do: val
  # defp unpack_8({:ok, << val::8 >>}), do: val
end
