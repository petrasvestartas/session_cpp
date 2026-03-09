#include "session.h"

using namespace session_cpp;

int main() {

    Session session;

    Mesh box = Mesh::create_box(1.0, 1.0, 1.0);
    // box.set_facecolors({Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(1,1,0), Color(1,0,1), Color(0,1,1)});
    box.set_objectcolor(Color(255,0,0));
    session.add_mesh(std::make_shared<Mesh>(box));







    std::string filepath = (std::filesystem::path(__FILE__).parent_path().parent_path() / "session_data" / "mesh_recheck.pb").string();
    session.pb_dump(filepath);

    return 0;
}
