defmodule VL53L0X.MixProject do
  use Mix.Project

  def project do
    [
      app: :nerves_vl53l0x,
      version: "0.1.0",
      elixir: "~> 1.10",
      build_embedded: Mix.env() == :prod,
      start_permanent: Mix.env() == :prod,
      description: description(),
      package: package(),
      deps: deps(),
      name: "VL53L0X",
      source_url: "https://github.com/OleMchls/nerves_vl53l0x"
    ]
  end

  # Run "mix help compile.app" to learn about applications.
  def application do
    []
  end

  # Run "mix help deps" to learn about dependencies.
  defp deps do
    [
      {:circuits_i2c, "~> 0.1"},
      {:ex_doc, "~> 0.14", only: :dev, runtime: false},
      {:dialyxir, "~> 1.0", only: [:dev], runtime: false}
    ]
  end

  defp description() do
    "Elixir library to interface with the VL53L0X Time-of-Flight sensor"
  end

  defp package() do
    [
      # This option is only needed when you don't want to use the OTP application name
      name: "nerves_vl53l0x",
      licenses: ["MIT"],
      links: %{"GitHub" => "https://github.com/OleMchls/nerves_vl53l0x"}
    ]
  end
end
