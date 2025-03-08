# minerzReefscape
2025 FRC code

# Library Information (2025 Updated)
### Installation
- [ ] Install NavX Library
- [ ] Install Phoenix Library:      
- https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-latest.json
- https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2025-latest.json
- [ ] Install REVLib.
- https://software-metadata.revrobotics.com/REVLib-2024.json
- Changelog:
https://docs.revrobotics.com/revlib/install/changelog
- [ ] Install ReduxLib.


Subsystem
- Hardware y métodos simples.
- Los comandos tienen vida, like the cola en ROS.
- Se pueden hacer cosas que se ocupen previo a ejecutar el comando. Por ejemplo en el swerve, se reinicia la odometría, o settear un cero en algún encoder.
- Supplier es como el nodo, es en enlace entre subsystem y command.
- addRequirements() es para que el command sepa que necesita el subsystem.
  - Si un comando necesita más de un subsystem, se pueden agregar más de un addRequirements.
- Los métodos initilize() y execute() son para que el command sepa qué hacer.
- Si no se sigue usando el comando se puede llamar al método end() para que se detenga.
- El método isFinished() es como para controlado, así como condiciones.
En robot container.
- configureCommands() y configureButtonBindings() es para que el robot conecte todo.
- La telemetría maneja el Dashboard.
- Cuando se quieran hacer dos coassa al mismo tiempo se ocupa hace unconanmdo paralelo