# import matplotlib.pyplot as plt
# import numpy as np

# class TelemetryManager:
#     def __init__(self):
#         self.telemetry_data = []
#         self.current_step = 0
#         self.data = {}
#         self.built = []
#         self.all_keys = set()
    
#     def submit(self, name, data):
#         self.data[name] = data
#         self.all_keys.add(name)
    
#     def step(self, timestamp):
#         entry = {"timestamp": timestamp, **self.data}
#         self.telemetry_data.append(entry)
#         self.data = {}
#         self.current_step += 1

#     def build_arrays(self):
#         for key in self.all_keys:
            

#     def draw_graph(self, key: callable):
#         fig, ax = plt.subplots()
#         for key in self.telemetry_data[0].keys():
#             if key != "timestamp":
#                 y_values = [entry[key] for entry in self.telemetry_data]
#                 x_values = [entry["timestamp"] for entry in self.telemetry_data]
#                 ax.plot(x_values, y_values, label=key)
#         ax.set_xlabel("Time")
#         ax.set_ylabel("Value")
#         ax.legend()
#         plt.savefig("telemetry_graph.png")


# manager = TelemetryManager()