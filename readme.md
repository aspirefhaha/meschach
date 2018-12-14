# SpaceTrek Pilot Demo Code


* Use Meschach Library Version 1.2b
* Update NaviGuide C array script

```shell
# in vim
% s/\(\d\+,\d\+,\d\+\.\d\+,\d\+,\d\+\.\d\+\)/\t{\1},/g
```

* Update Control Gain C array script

```shell
% s/\(\d\+\.\?\d*,\d\+\.\?\d*,\d\+\.\?\d*,\d\+\.\?\d*,\d\+\.\?\d*\)/\t{\1},/g
```

* Update IMU Data C array script
```SHELL
## single data replace % s/\(-\?\d\+[\.]\?\d*E\?-\?\d\+\)/\t{\1},/g
% s/\(-\?\d\+\.\?\d*E\?-\?\d*,-\?\d\+\.\?\d*E\?-\?\d*,-\?\d\+\.\?\d*E\?-\?\d*,-
\?\d\+\.\?\d*E\?-\?\d*,-\?\d\+\.\?\d*E\?-\?\d*,-\?\d\+\.\?\d*E\?-\?\d*,-\?\d\+\.
\?\d*E\?-\?\d*\)/\t{\1},/g
```

## need bind parameters
  - navi paratmeters:
    - A0
    - B0
    - LAMBDA0
    - H0
    - Faie0
    - Miu0
    - init Posture:
      - fai0
      - psi0
      - gamma0
    - H1
    - density of atmosphere
      - H1
      - H2
      - row1
      - row2
    - temperature of air parameters(Ta)
  - guidance coefficents
    - k1fai,k2fai,k1psi,k2psi
  - flying time series
    - time action height acceleration thetat speed(mach)
  - Trajectory
    - DeltaTime
    - Hcx
    - Thetacx

## Output Data to RemoteCapture
  - Navi result
    - deltaTime
    - B
    - Lambda
    - H
    - VI (velocity in I)
    - MovementI
    - kineticPressure
    - Mach
    - AirPressure
    - Aim Cnb(Only Aim State)
    - CgI ( Only Aim State)
    - CgE ( Only Aim State)
    - CTE ( Only Aim state)
    - CIb
    - Cnb
    - psi,fai,gamma
    - r
    - state( init / subsystem/simulation/.../reallaunching)
    - substate (aim/flying/...)
    - Acceleration(X,Y,Z)
    - Omega(X,Y,Z)
    - thetaT
  - Guidance result
    -
