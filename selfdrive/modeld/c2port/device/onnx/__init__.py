"""Stand-in for the onnx package on the comma two: loads graphs exported by export_graph_pickle.py."""
import pickle
def load(f):
  return pickle.load(f if hasattr(f, 'read') else open(f, 'rb'))
