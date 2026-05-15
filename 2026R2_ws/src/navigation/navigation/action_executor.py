import time

class ActionExecutor:
    def __init__(self, logger):
        self.logger = logger
        self.status = "idle" # idle, running, done
        self.start_time = 0
        self.current_action = None
        self.action_queue = []

    def set_actions(self, actions):
        self.action_queue = list(actions)
        if not self.action_queue:
            self.status = "done"
        else:
            self.status = "running"
            self._start_next_action()

    def _start_next_action(self):
        if not self.action_queue:
            self.status = "done"
            return

        self.current_action = self.action_queue.pop(0)
        self.action_type = self.current_action.get('type', 'wait')
        self.start_time = time.time()
        self.logger.info(f"Executing action: {self.action_type}")

    def update(self):
        if self.status != "running":
            return

        if self.action_type == "wait":
            duration = self.current_action.get('duration_s', 0.0)
            if time.time() - self.start_time >= duration:
                self._start_next_action()
        else:
            # Placeholder for other actions
            self.logger.warn(f"Unknown action type: {self.action_type}. Skipping.")
            self._start_next_action()

    def is_done(self):
        return self.status == "done"
