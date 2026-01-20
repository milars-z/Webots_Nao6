
from controller import Robot

class NaoStander(object):
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Noms des moteurs du Nao
        self.motor_names = [
            "HeadYaw", "HeadPitch",
            "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
            "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw",
            "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
            "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"
        ]
        self.motors = {}
        for name in self.motor_names:
            self.motors[name] = self.robot.getDevice(name)

    def stand_up(self):
        """Définit les moteurs sur une posture debout."""
        # Angles cibles pour la posture debout (en radians)
        # Ces valeurs peuvent nécessiter des ajustements en fonction de la version spécifique du Nao et de l'environnement
        target_positions = {
            "HeadYaw": 0.0,
            "HeadPitch": 0.0,
            "LShoulderPitch": 1.5,
            "LShoulderRoll": 0.15,
            "LElbowYaw": -1.0,
            "LElbowRoll": -0.5,
            "LWristYaw": 0.0,
            "RShoulderPitch": 1.5,
            "RShoulderRoll": -0.15,
            "RElbowYaw": 1.0,
            "RElbowRoll": 0.5,
            "RWristYaw": 0.0,
            "LHipYawPitch": 0.0,
            "LHipRoll": 0.0,
            "LHipPitch": -0.25,
            "LKneePitch": 0.5,
            "LAnklePitch": -0.25,
            "LAnkleRoll": 0.0,
            "RHipYawPitch": 0.0,
            "RHipRoll": 0.0,
            "RHipPitch": -0.25,
            "RKneePitch": 0.5,
            "RAnklePitch": -0.25,
            "RAnkleRoll": 0.0
        }

        for name, position in target_positions.items():
            if name in self.motors:
                self.motors[name].setPosition(position)

    def run(self):
        """Boucle principale du contrôleur."""
        print("Mise du robot Nao en position debout.")
        self.stand_up()

        # Boucle pour maintenir la position
        while self.robot.step(self.timestep) != -1:
            pass

# Créer une instance du contrôleur et l'exécuter
controller = NaoStander()
controller.run()