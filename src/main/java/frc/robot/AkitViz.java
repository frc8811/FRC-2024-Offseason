package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/// Class helps visualize the motion of the components in *Advantage Scope*.
/// It maintains a TF tree and stores it in a sorted list.
/// @author [ZhangzrJerry](https://github.com/zhangzrjerry)
/// @Usage:
/// 1. Update robot models and config.json in AScope, see
///    [docs](https://docs.advantagescope.org/more-features/custom-assets#articulated-components)
/// 2. Update Ids in `AkitViz.java`:
///    ```java
///    public AkitViz {
///      public static final class Ids {
///        public static final class ExampleSubsystem {
///          public static final int EXAMPLE_COMPONENT_1 = 0;
///          public static final int EXAMPLE_COMPONENT_2 = 1;
///        }
///      }
///    }
///    ```
/// 3. Call `AkitViz.periodic()` periodically in `Robot.java`:
///    ```java
///    public class Robot {
///      void robotPeriodic() {
///        if(Config.MODE != Config.Mode.REAL) {
///          AkitViz.getInstance().periodic();
///        }
///      }
///    }
///    ```
/// 4. Define the zeroed transform matrix in `SubsystemConfig.java`:
///    ```java
///    public class SubsystemConfig {
///      public static final Transform3d ZERO_POSE;
///    }
///    ```
/// 5. Register a `record` object in the constructor of `Subsystem.java`:
///    ```java
///    public class Subsystem {
///      public Subsystem() {
///        AkitViz.getInstance().register(
///          new AkitViz.AkitVizComponent(
///            AkitViz.Ids.Subsystem.COMPONENT,
///            AkitViz.Ids.Subsystem.PARENT_COMPONENT,
///            () -> SubsystemConfig.ZERO_POSE.plus(this.deltaPose)  // or this.getDeltaPose()
///          )
///        );
///      }
///    }
///    ```
public class AkitViz {
  /**
   * The id should range from 0 to n-1 and be consistent with the part order of config.json in
   * AScope. -1 means robot frame.
   */
  public static final class Ids {
    public static final class Swerve {
      public static final int FL_WHEEL = 0;
      public static final int BL_WHEEL = 1;
      public static final int BR_WHEEL = 2;
      public static final int FR_WHEEL = 3;
    }
  }

  /**
   * @param componentId the id of the component, ∈[0,N]
   * @param parentId the id of the parent component, ∈[-1,componentId), -1 means robot frame
   * @param transformSupplier the supplier of the transform matrix from the parent component to the
   *     current component
   */
  public record AkitVizComponent(
      int componentId, int parentId, Supplier<Transform3d> transformSupplier) {
    public AkitVizComponent {
      // check the validity of the parameters
      if (componentId < 0) {
        throw new IllegalArgumentException("componentId out of index");
      }
      if (parentId < -1) {
        throw new IllegalArgumentException("parentId out of index");
      }
      if (isStrictBigEndian && parentId >= componentId) {
        throw new IllegalArgumentException(
            "componentId should not greater than or equal to parentId");
      }
      if (transformSupplier == null) {
        throw new IllegalArgumentException("transformSupplier should not be null");
      }
    }
  }

  /**
   * Register the component to the AkitViz
   *
   * @param akitVizComponent the component to be registered, should be declared in the constructor
   *     of the subsystem
   * @usage see {@link AkitViz}
   */
  public void register(AkitVizComponent akitVizComponent) {
    components.add(akitVizComponent);
    isSort = false;
  }

  /**
   * Should be called periodically to update the poses of the components
   *
   * @usage see {@link AkitViz}
   */
  public void periodic() {
    Pose3d[] poses = new Pose3d[components.size()];
    Transform3d[] tfs = new Transform3d[components.size()];

    if (isStrictBigEndian) {
      if (!isSort) {
        components.sort(Comparator.comparingInt(AkitVizComponent::componentId));
        isSort = true;
      }

      // check the validity of registered id
      for (int i = 0; i < components.size(); i++) {
        if (components.get(i).componentId != i) {
          ++waitRegFailCount;
          if (waitRegFailCount > 50) {
            throw new RuntimeException(
                "Timed out waiting for component registration. "
                    + "The component ID should be continuous from 0 to n-1. "
                    + "Please check if there is any gap in the middle. ");
          }
          return;
        }
      }

      for (int i = 0; i < components.size(); ++i) {
        // get transform matrix from supplier
        tfs[i] = components.get(i).transformSupplier.get();

        // propagate the transform matrix
        if (components.get(i).parentId != -1) {
          tfs[i] = components.get(components.get(i).parentId).transformSupplier.get().plus(tfs[i]);
        }
        poses[i] = new Pose3d().plus(tfs[i]);
      }
    } else {
    }

    Logger.recordOutput(TOPIC_NAME, poses);
  }

  private static Boolean isSort = true;
  private static Integer waitRegFailCount = 0;
  private static final List<AkitVizComponent> components = new ArrayList<>();

  private static final Boolean isStrictBigEndian = true;
  private static final String TOPIC_NAME = "Visualize/Component";

  private static AkitViz instance;

  private AkitViz() {}

  public static AkitViz getInstance() {
    if (instance == null) {
      instance = new AkitViz();
    }
    return instance;
  }
}
