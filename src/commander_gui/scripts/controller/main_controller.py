
class MainController:
    def __init__(self, main_model):
        self.main_model = main_model

    def get_data(self, tag_id):
        return self.main_model.get_data(tag_id)
