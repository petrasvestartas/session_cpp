#include "mini_test.h"
#include "file_step.h"
#include <filesystem>

#ifdef SESSION_WITH_OCCT
#error Native BRep acceptance must be built without OCCT
#endif

int main() {
    const int failures = session_cpp::mini_test::run_all("cpp");
    const std::filesystem::path root = std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::filesystem::path folder = root / "build/native-geometry-tests";
    std::filesystem::create_directories(folder);
    std::filesystem::copy_file(root.parent_path() / "session_tests/session_cpp/brep_classify_test.json",
        folder / "classification_tests.json", std::filesystem::copy_options::overwrite_existing);
    const std::vector<session_cpp::BRep> chairs = session_cpp::file_step::read_file_step_breps(
        (root / "tests/fixtures/solidbrep.stp").string());
    for (size_t i = 0; i < chairs.size(); ++i) {
        const std::string name = "chair_" + std::to_string(i);
        chairs[i].file_json_dump((folder / (name + ".brep.json")).string());
        chairs[i].mesh().file_json_dump((folder / (name + ".mesh.json")).string());
    }
    return failures ? 1 : 0;
}
