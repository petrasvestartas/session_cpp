#include "mini_test.h"

#ifdef SESSION_WITH_OCCT
#error Native intersection acceptance must be built without OCCT
#endif

int main() {
    return session_cpp::mini_test::run_all("cpp") ? 1 : 0;
}
