---
layout: default
title: Femto Amplifier
nav_order: 13
---


# Femto Amplifier Support

Support for [Femto](http://www.femto.de/) brand low-noise current amplifiers.
Two independent driver approaches are available in the std module.


## SNL-Based Driver (`femto.st`)

The State Notation Language program `femto.st` controls Femto DLPCA-200
amplifiers by translating a gain index (0-14) to the appropriate combination
of digital output bits. It supports both high-speed and low-noise modes.

### Gain Table (DLPCA-200)

| Index | Gain (V/A) | Mode |
|-------|-----------|------|
| 0 | 10^3 | High Speed |
| 1 | 10^4 | High Speed |
| 2 | 10^5 | High Speed |
| 3 | 10^6 | High Speed |
| 4 | 10^7 | High Speed |
| 5 | 10^8 | High Speed |
| 6 | 10^9 | High Speed |
| 7 | 10^3 | Low Noise |
| 8 | 10^4 | Low Noise |
| 9 | 10^5 | Low Noise |
| 10 | 10^6 | Low Noise |
| 11 | 10^7 | Low Noise |
| 12 | 10^8 | Low Noise |
| 13 | 10^9 | Low Noise |
| 14 | 10^9 | Low Noise |

### Database

`femto.db` provides the following records:

| Record Name | Type | Purpose |
|-------------|------|---------|
| `$(P)$(H)$(F):Level` | mbbo | Gain index selection (0-14) |
| `$(P)$(H)$(F):Gain` | ao | Gain value in V/A |
| `$(P)$(H)$(F):Name` | stringout | Amplifier description |

### Loading with femto.iocsh

The simplest way to load the SNL-based femto support is with the iocsh script:

```
iocshLoad("$(STD)/iocsh/femto.iocsh", "PREFIX=xxx:, HARDWARE=A1, FUNC=femto, G1_PV=xxx:Unidig1Bo0, G2_PV=xxx:Unidig1Bo1, G3_PV=xxx:Unidig1Bo2, NO_PV=xxx:Unidig1Bo3, STD=$(STD)")
```

### Loading Manually

```
dbLoadRecords("$(STD)/stdApp/Db/femto.db", "P=xxx:,H=A1,F=femto,G1=xxx:Unidig1Bo0,G2=xxx:Unidig1Bo1,G3=xxx:Unidig1Bo2,NO=xxx:Unidig1Bo3")
doAfterIocInit("seq &femto, 'P=xxx:,H=A1,F=femto,G1=xxx:Unidig1Bo0,G2=xxx:Unidig1Bo1,G3=xxx:Unidig1Bo2,NO=xxx:Unidig1Bo3'")
```

### Autosave

- `femto.req` -- basic request file
- `femto_settings.req` -- settings request file


## Transform-Based Drivers (`femto_DxPCA_x00`)

An alternative set of drivers uses `transform` records for gain control instead
of a State Notation Language program. These drivers are independent of the
SNL-based driver above.

Model-specific database files are available:

| File | Model |
|------|-------|
| `femto_DDPCA_300.db` | Femto DDPCA-300 |
| `femto_DHPCA_100.db` | Femto DHPCA-100 |
| `femto_DLPCA_200.db` | Femto DLPCA-200 |

### Macros

The macros specify which digital output PVs control each function:

| Macro | Description | Used By |
|-------|-------------|---------|
| `P` | PV prefix | All models |
| `A` | Amplifier name | All models |
| `G1` | Gain first (lowest) bit PV | All models |
| `G2` | Gain second bit PV | All models |
| `G3` | Gain third bit PV | All models |
| `G4` | Gain fourth bit PV | DDPCA-300 only |
| `SN` | High speed / Low noise bit PV | DHPCA-100, DLPCA-200 |
| `C` | Coupling PV | DHPCA-100, DLPCA-200 |
| `F10` | 10 MHz Filter PV | DHPCA-100 only |
| `F1` | 1 MHz Filter PV | DHPCA-100 only |

### Substitution File Example

Create a substitution file (e.g., `femto_DxPCA_x00.substitutions`) to load the
appropriate database for your amplifier model:

```
# Uncomment the femto amplifier you want to use.

#file "$(STD)/stdApp/Db/femto_DDPCA_300.db"
#{
#pattern
#{P       A       G1          G2          G3          G4         }
#{crate:  femto1  Unidig1Bo0  Unidig1Bo1  Unidig1Bo2  Unidig1Bo3 }
#}

#file "$(STD)/stdApp/Db/femto_DLPCA_200.db"
#{
#pattern
#{P       A       G1          G2          G3          SN          C          }
#{crate:  femto2  Unidig1Bo4  Unidig1Bo5  Unidig1Bo6  Unidig1Bo7  Unidig1Bo8 }
#}

#file "$(STD)/stdApp/Db/femto_DHPCA_100.db"
#{
#pattern
#{P       A       G1          G2           G3           SN           C            F10          F1          }
#{crate:  femto3  Unidig1Bo9  Unidig1Bo10  Unidig1Bo11  Unidig1Bo12  Unidig1Bo13  Unidig1Bo14  Unidig1Bo15 }
#}
```

### Autosave

- `femto_DDPCA_300.req`
- `femto_DHPCA_100.req`
- `femto_DLPCA_200.req`
