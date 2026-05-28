#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <mujoco/mujoco.h>

namespace {

void PrintUsage(const char* program) {
  std::fprintf(stderr, "Usage: %s input.urdf output.xml\n", program);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3 ||
      std::strcmp(argv[1], "-h") == 0 ||
      std::strcmp(argv[1], "--help") == 0) {
    PrintUsage(argv[0]);
    return argc == 2 ? EXIT_SUCCESS : EXIT_FAILURE;
  }

  const char* input = argv[1];
  const char* output = argv[2];
  char error[4096] = {0};

  // mj_loadXML stores the parsed mjSpec needed later by mj_saveLastXML.
  mjModel* model = mj_loadXML(input, nullptr, error, sizeof(error));
  if (!model) {
    std::fprintf(stderr, "Failed to load '%s': %s\n", input, error);
    return EXIT_FAILURE;
  }

  if (error[0] != '\0') {
    std::fprintf(stderr, "MuJoCo warning while loading '%s': %s\n", input, error);
    error[0] = '\0';
  }

  const int ok = mj_saveLastXML(output, model, error, sizeof(error));
  mj_deleteModel(model);

  if (!ok) {
    std::fprintf(stderr, "Failed to save '%s': %s\n", output, error);
    return EXIT_FAILURE;
  }

  std::printf("Saved MJCF: %s\n", output);
  return EXIT_SUCCESS;
}
