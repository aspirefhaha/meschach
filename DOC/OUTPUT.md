# Output Data to RemoteCapture
  - uptime
  - state( init / subsystem/simulation/.../reallaunching)
  - substate (aim/flying/...)
  - Navi result
    - B (2.3.2)
    - Lambda  (2.3.2)
    - H (3.3)
    - VI (velocity vector in I) (3.3)
    - MovementI (3.3)
    - kineticPressure ()
    - Mach
    - AirPressure
    - Aim Cnb(Only Aim State)
    - CgI ( Only Aim State) (2.3.2)
    - CgE ( Only Aim State) (2.3.2)
    - CTE ( Only Aim state) (2.3.2)
    - CIb (3.4)
    - Cnb (3.2)
    - psi,fai,gamma (3.4)
    - r (di xin shiliang) (3.3)
    - Acceleration(X,Y,Z) ()
    - Omega(X,Y,Z)
    - thetaT (3.3)
  - Guidance result
    - instant
      - Tdz (from uptime)
      - Tdh (from uptime)
      - T0  (from uptime)
      - T0+1  (from T0) Tqk
      - T0+2.5 (from T0) Tzw
      - Trq (from T0)
      - Thj (from T0)
      - Tfl (from T0)
      - Tzg (from T0)
      - Position(zg)
      - Tld
      - Position(ld)
    - continue
      - deltatime (from T0)
      - guideNormal
      - guideLand
      - trajetory H,theta,psic,phic,gammac??
      - calc theta ,epsilon,alpha,beta
      - posture angle diff
      - rudder control signal (3 + 4 direction) ? ori or limit 
