# Spac [![Test Spac](https://github.com/FSIPLEIRIA/Spac/actions/workflows/test_wf.yaml/badge.svg)](https://github.com/FSIPLEIRIA/Spac/actions/workflows/test_wf.yaml)

No not the Finance Derivate, Simple PID Ackerman Controll

It works by aiming then throtling/braking.  Both controls are operated with separate PID controllers

## Setup ["native"]

The package requires diagnostic-updater, geographic_info, ackermann_msg, and angles package

## Setup [Arch based]

```bash
paru -S ros2-humble-ackermann-msgs
```
