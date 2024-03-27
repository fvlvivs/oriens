# oriens_madgwick

This package implements a Madgwick filter where both IMU and MARG versions are available.

### Implementation details

Note: here the not-simplified version of the filter is implemented, which involves:
- tuning two parameters, $\mu$ and $\gamma$;
- freedom to set gravity and magnetic reference fields.