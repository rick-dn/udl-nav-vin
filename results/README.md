# Results

Plots and numbers worth keeping, one folder per stage. Raw recordings stay in `runs/` (not in git); a result here is copied from there by hand once it's final.

| Folder | What |
|---|---|
| [baseline/](baseline/) | OpenVINS on the clean sim, scripted square flight |
| [flight_set/](flight_set/) | OpenVINS on the four test flights: easy, medium, hard, weak motion |
| [faults/](faults/) | The same kind of flight replayed with injected faults: IMU noise and bias, time offset, dropped / frozen / dark / blurred frames |
