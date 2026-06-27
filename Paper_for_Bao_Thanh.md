
# Visual Servoing Policy

## Input

### Config (One-Time)

| Quantity | Size | Type | Source | Description |
| :--- | :--- | :--- | :--- | :--- |
| 2 | 3×3 + 3×1 each | Extrinsic Matrix | Hardcoded config | R (rotation) and t (translation): side camera optical frame → center camera optical frame |
| 1 | 3x3 | Intrinsic Matrix | Hardcoded config | of three cameras identical |
| 3 | 1152 x 1024 (px) | Source Image | ROS topic | left, middle and right camera |

### Per Iteration

| Quantity | Size | Type | Source | Description |
| :--- | :--- | :--- | :--- | :--- |
| 1 | 3x1 + quaternion | Pose | TF2 | of middle camera’s optical reference to `base_link` |
| 1 | 3x1 | Position | TF2 | gripper/tcp position in `base_link` |
| 3 | 1152 x 1024 (px) | Query Image | ROS topic | left, middle and right camera |

## Output

| Quantity | Size | Type | Description |
| :--- | :--- | :--- | :--- |
| 1 | 6×1 | Controlling Twist `[vx, vy, vz, ωx, ωy, ωz]` | Controlling twist of gripper/TCP expressed in `base_link` frame |

## System Architecture

```mermaid  
flowchart 
classDef large font-size:10px;

A["<b>Source Keypoint Extractor</b><br/>(SuperPoint)"]  
B["<b>Feature Matching</b><br/>(LightGlue)"]  
C["<b>IBVS Controller</b>"]  
D["<b>State Machine</b>"]  
E["<b>Motion Update</b><br/>(Twist)"]  

class A,B,C,D,E large

A --> B --> C --> D --> E
```

## Source Images Keypoint Extraction (One-Time)

```mermaid
flowchart TD
    A["Load Source Images<br/>Left, Center, Right<br/>(1152×1024 px)"] --> B["SuperPoint Feature<br/>Extraction (per image)"]
    B --> C["SuperGlue<br/>Matching"]
    C --> C1["center ↔ left<br/>keypoint match"]
    C --> C2["center ↔ right<br/>keypoint match"]
    C1 --> D1["MAGSAC Homography<br/>Outlier Filtering<br/>(ransacReprojThreshold=3.5)"]
    C2 --> D2["MAGSAC Homography<br/>Outlier Filtering<br/>(ransacReprojThreshold=3.5)"]
    D1 --> E1["Store: center_left ↔ left<br/>stereo correspondences"]
    D2 --> E2["Store: center_right ↔ right<br/>stereo correspondences"]
    E1 --> F["Cache source keypoints<br/>in pixel coordinates"]
    E2 --> F
    F --> G["Pre-extract SuperPoint<br/>features from all 3<br/>source images for reuse"]
    G --> H["Free SourceKpts<br/>(GPU memory)"]
```

*Key design*:<br/>• SourceKpts uses a separate SuperPoint+SuperGlue instance that is discarded after extraction to free GPU memory.<br/>• FeatureMatching then uses a fresh SuperPoint+LightGlue instance for the online matching loop.

## 3. Feature Matching Extraction (Trinocular, per Iteration)

```mermaid
flowchart TD
    subgraph "3× SuperPoint Extraction (CUDA Stream Overlap)"
        A1["Live Left Image"] --> B1["SuperPoint Extract<br/>(stream_L)"]
        A2["Live Center Image"] --> B2["SuperPoint Extract<br/>(stream_C)"]
        A3["Live Right Image"] --> B3["SuperPoint Extract<br/>(stream_R)"]
    end

    B1 --> C1["Wait stream_L"]
    B2 --> C2["Wait stream_C"]
    B3 --> C3["Wait stream_R"]

    C1 --> D1["LightGlue Match<br/>Left Live ↔ Left Source<br/>(source feats precomputed)"]
    C3 --> D2["LightGlue Match<br/>Right Live ↔ Right Source<br/>(source feats precomputed)"]
    C2 --> D3["LightGlue Match<br/>Center Live ↔ Center Source<br/>(source feats precomputed)"]

    D1 --> E1["Filter: match_specific_kpts<br/>→ src left keypoints only"]
    D2 --> E2["Filter: match_specific_kpts<br/>→ src right keypoints only"]
    D3 --> E3["Filter center_left keypoints"]
    D3 --> E4["Filter center_right keypoints"]

    E1 --> F1["Validity check:<br/>both center_left AND left<br/>must be valid (not NaN)"]
    E3 --> F1
    E2 --> F2["Validity check:<br/>both center_right AND right<br/>must be valid (not NaN)"]
    E4 --> F2

    F1 --> G1["Left Stereo Pairs<br/>(center_left_src, left_src)<br/>vs (center_left_live, left_live)"]
    F2 --> G2["Right Stereo Pairs<br/>(center_right_src, right_src)<br/>vs (center_right_live, right_live)"]
```


