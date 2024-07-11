class ConstraintChecker:

    @staticmethod
    def ValidTrajectory(combined_trajectory):
        return True
    
    class Result:
        def __init__(self, valid, message):
            self.valid = valid
            self.message = message