## Testing Strategy

Testing is performed at multiple levels:
- Unit testing of modules
- Integration testing of subsystems
- System-level validation

## Testability

The architecture supports testability through:
- Clear interfaces
- Dependency injection
- Hardware abstraction

## Continuous Verification

Automated tests are executed regularly
to detect regressions early.

```mermaid
stateDiagram-v2
    Idle : Waiting for external events
    Active : Processing sensor data
    Fault : Error detection and recovery

    Idle --> Active : event
    Active --> Fault : error
    Fault --> Safe

    click Idle "#requirements" "Idle state requirements"
    click Active "#architecture" "Event handling architecture"
    click Fault "#dependability-safety" "Fault handling and safety analysis"
```