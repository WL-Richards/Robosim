namespace hal {

class HandleBase {
 public:
  HandleBase();
  HandleBase(const HandleBase&) = delete;
  HandleBase& operator=(const HandleBase&) = delete;
  virtual ~HandleBase();

  static void ResetHandles();
  static void ResetGlobalHandles();
};

HandleBase::HandleBase() = default;

HandleBase::~HandleBase() = default;

void HandleBase::ResetHandles() {}

void HandleBase::ResetGlobalHandles() {}

}  // namespace hal
