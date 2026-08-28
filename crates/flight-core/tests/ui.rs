#[test]
fn compile_fail_illegal_api() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/*.rs");
}
