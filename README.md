
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

## 1. System Architecture

```mermaid  
flowchart 
classDef large font-size:10px;

A["<b>Source Feature Matching</br>& Source Keypoint Extraction</b><br/>(SuperPoint+SuperGlue)"]  
B["<b>Live Feature Matching <br/>& Stereo Pair Filtering</b><br/>(SuperPoint+LightGlue)"]  
C["<b>3D point Computation</b></br>(Stereo Triangulation)"]
D["<b>IBVS Controller</b>"]  
E["<b>State Machine</b>"]  
F["<b>Motion Update</b><br/>(Twist)"]  

class A,B,C,D,E,F large

A --> B --> C --> D --> E --> F
```

## 2. Source Images Keypoint Extraction (One-Time)

```mermaid
flowchart TD
    A["Source <b>Image Left</b><br/>(1152×1024 px)"]
    A1["Source <b>Image Center</b><br/>(1152×1024 px)"]
    A2["Source <b>Image Right</b><br/>(1152×1024 px)"]

    A --> B["<b>Source Center Image</b><br/>Feature Extraction<br/>(SuperPoint)"]
    A1 --> B1["<b>Source Left Image</b><br/>Feature Extraction<br/>(SuperPoint)"]
    A2 --> B2["<b>Source Right Image</b><br/>Feature Extraction<br/>(SuperPoint)"]

    C["Matching Feature</br><b>Left ↔ Center</b></br>(SuperGlue)"]
    C1["Matching Feature</br><b>Center ↔ Right</b></br>(SuperGlue)"]
    B --> C
    B1 --> C    
    B2 --> C1
    B --> C1

    C --> D["MAGSAC Homography<br/><b>Outlier Filtering</b><br/>(ransacReprojThreshold=3.5)"]
    C1 --> D1["MAGSAC Homography<br/><b>Outlier Filtering</b><br/>(ransacReprojThreshold=3.5)"]
    
    D --> E["Matched<br/><b>Source Left</b><br/>Keypoints</br>(Pixel coordinates)"]
    D --> E1["Matched<br/><b>Source Center ↔ Left</b><br/>Keypoints</br>(Pixel coordinates)"]
    D1 --> E2["Matched<br/><b>Source Center ↔ Right</b><br/>Keypoints</br>(Pixel coordinates)"]
    D1 --> E3["Matched<br/><b>Source Right</b><br/>Keypoints</br>(Pixel coordinates)"]

    F["Store</br><b>Stereo Correspondence Source Keypoints</b><br/>in pixel coordinates"]
    E --> F
    E1 --> F
    E2 --> F
    E3 --> F
    
```

*Key design*:<br/>• SourceKpts uses a separate SuperPoint+SuperGlue instance that is discarded after extraction to free GPU memory.<br/>• FeatureMatching then uses a fresh SuperPoint+LightGlue instance for the online matching loop.

## 3. Feature Matching & Stereo Pair Filtering (per Iteration)

![My Image](Charts/Feature_Matching_&_Stereo_Pair_Filtering.drawio.png)

## 4. Stereo Triangulation → 3D Feature Points

```mermaid
flowchart TD

    subgraph S1["Camera Geometry"]
        subgraph S11["<b>Pre-computed</b>"]
            A["<b>Stereo Keypoint Pairs</b><br/>(Center, Side)"]
        end
        B["<b>Invert Extrinsics</b><br/>R = Rᵀ<br/>t = −Rᵀt"]
        C["<b>Build Projection Matrices</b><br/>P<sub>center</sub> = K[I | 0]<br/>P<sub>side</sub> = K[R | t]"]
        A --> B --> C
        style S11 fill:#4285f4,stroke:#333,stroke-width:2px
    end

    subgraph S2["Triangulation"]
        D["<b>Triangulate 3D Points</b><br/>cv2.triangulatePoints()"]
        E["<b>Homogeneous</b></br>↓</br><b>Euclidean</b><br/>"]
        C --> D --> E
    end

    subgraph S3["Depth & Normalization"]
        F["<b>Extract Depth Z</b><br/>from euclidean coordinates "]
        G["<b>Normalize Image Coordinates</b><br/>x = (u − c<sub>x</sub>)/f<sub>x</sub><br/>y = (v − c<sub>y</sub>)/f<sub>y</sub>"]
        E --> F --> G
    end

    subgraph S4["3D Reconstruction"]
        H["<b>Compute 3D Point</b><br/>X = (x × Z, y × Z, Z)"]
        G --> H
    end
```

## 5. IBVS Control

