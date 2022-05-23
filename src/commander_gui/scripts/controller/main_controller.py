
class MainController:
    def __init__(self, robot_model, main_model):
        self.robot_model = robot_model
        self.main_model = main_model

    def get_data(self, tag_id):
        return self.main_model.get_data(tag_id)

    def connect_to_robot(self):
        self.robot_model.connect_to_robot()