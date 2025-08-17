# Servo hydraulic actuator

Notebook for design of mechanism to provide control of pressure in a closed hydraulic system

Motor -> mechanical drive -> master cylinder

GH pages:

Binder: 

## Components
### Motor:
 - Brushed DC motor
 - Voltage limit on controller
 - Simulation of transient thermal characteristics based off motor supplier data

### Mechanical Drive:
 - Gearbox
 - Rotary to linear mechanism

### Hydraulic system
- Stiffness represented as a linear spring based on master cylinder travel

### Controller
- Simple PI controller controlling pressure by varying motor voltage

## Assumptions
- Winding resistance change with temperature is ignored. This will over-estimate performance of motor at continuous / heavy use
- Non-linear (detailed entrained / dissolved air) behaviour of fluid ignored. Low pressure behaviour of fluid will be inaccurate
