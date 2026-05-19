Import("env")

def skip_midi_files(node):
    if "AudioGeneratorMIDI" in node.get_abspath():
        return None
    return node

env.AddBuildMiddleware(skip_midi_files, "*.cpp")
