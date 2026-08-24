# m_shield_demo 與 ivp/src 核心模組技術架構與演算法說明文件

本文件針對 **m_shield_demo（Shield AI 協同搜索、包夾與驅逐防衛任務）** 與 **ivp/src 核心程式庫** 的最新演算法、模組重構與架構設計，提供完整且深入的技術說明。

---

# 一、 系統架構概觀與新增/修改檔案詳解

本系統採用 **MOOS-IvP** 分散式發布/訂閱（Pub/Sub）架構，由 5 個獨立 MOOS 社群（Shoreside, Abe, Ben, Target, Mothership）組成。

`mermaid
graph TD
    subgraph ivp_src [ivp/src 核心模組庫]
        pRD[pRegionDivider<br/>幾何主軸對齊/分區切割/覆蓋網格]
        pTC[pTargetCoordinator<br/>自主接戰狀態機/逃逸路徑評估/阻擋弧包夾]
        pTPP[pTargetPathPlanner<br/>入侵目標即時路徑規劃]
        BAC[BHV_AvdColregsV22<br/>COLREGS 國際避碰規則]
        BTR[BHV_Trail<br/>絕對方位包夾與航跡自動清理]
        PMV[pMarineViewer / PMV_GUI<br/>階層式下拉選單與圖台可視化]
    end

    subgraph m_shield_demo [m_shield_demo 任務配置]
        MS[meta_shoreside.moos<br/>岸端操作介面與指令中心]
        MM[meta_mothership.moos<br/>母船戰術協調運算中樞]
        MV[meta_vehicle.moos / bhv<br/>子船 abe, ben 自主巡航/夾擊]
        MT[meta_target.moos / bhv<br/>目標 target 巡邏與受迫避碰]
        L[launch.sh / launch_*.sh<br/>多進程排程啟動腳本]
    end

    ivp_src --> m_shield_demo
`

### 1. ivp/src/ 核心模組與演算法庫

| 模組 / 原始碼檔案 | 檔案角色與核心修改 | 詳細功能與職責說明 |
| :--- | :--- | :--- |
| **pRegionDivider** | [RegionDivider.h](file:///l:/home/yoei/moos-ivp/ivp/src/pRegionDivider/RegionDivider.h)<br>[RegionDivider.cpp](file:///l:/home/yoei/moos-ivp/ivp/src/pRegionDivider/RegionDivider.cpp) | **區域搜索劃分與航線生成中樞**：<br>1. **凸包建立**：接收岸端滑鼠標點（REGION_VERTEX）動態計算 Convex Hull。<br>2. **主軸對齊（Sweep Alignment）**：以旋轉卡尺計算最小外接矩形（OBB）長軸，航線沿長軸鋪設，減少 180° 急轉彎。<br>3. **跨航向均分（Cross-track Slicing）**：垂直於長軸切成 $ 等份 Band，保留完整航道長度。<br>4. **多樣化掃描模式**：支援 Lawnmower、Skip（跳行防急轉）、Spiral（向心矩形螺旋）、Perimeter（周界巡邏）。<br>5. **視距聯鎖與覆蓋著色**：根據 SENSOR_RADIUS 動態縮放航寬與端點內縮（Inset），維護 XYConvexGrid 增量發送 VIEW_GRID_DELTA。<br>6. **防禦邊界廣播**：發布 REGION_POLY 與 REGION_CENTER 供協調器使用。 |
| **pTargetCoordinator** | [TargetCoordinator.h](file:///l:/home/yoei/moos-ivp/ivp/src/pTargetCoordinator/TargetCoordinator.h)<br>[TargetCoordinator.cpp](file:///l:/home/yoei/moos-ivp/ivp/src/pTargetCoordinator/TargetCoordinator.cpp) | **自主接戰、防禦阻擋弧與包夾驅逐協調器**：<br>1. **自主閉環狀態機**：SEARCHING $\rightleftharpoons$ INTERCEPTING 自動循環，達成目標侵入自啟動、驅逐出境自動復歸搜索。<br>2. **逃逸路徑代價評估（exitCost）**：射線投影邊界距離結合目標當前航向偏角懲罰，評估最佳驅逐方向並加滯後防抖。<br>3. **防禦阻擋弧（Pincer Blocking Arc）**：在目標船與任務水域之間建立阻擋牆，計算子船絕對站位角度。<br>4. **船艏防護（Bow Guard）**：自動偏移正對目標船艏的危險站位，符合海事安全。<br>5. **非交叉指派與滯後（Hysteresis）**：確保子船不互穿航道，防止中心線擺盪打轉。<br>6. **成果統計**：維護 SHIELD_EVICTIONS（驅逐成功次數）與 SHIELD_DETECTIONS。 |
| **pTargetPathPlanner** | [TargetPathPlanner.h](file:///l:/home/yoei/moos-ivp/ivp/src/pTargetPathPlanner/TargetPathPlanner.h)<br>[TargetPathPlanner.cpp](file:///l:/home/yoei/moos-ivp/ivp/src/pTargetPathPlanner/TargetPathPlanner.cpp) | **入侵目標自訂航線規劃器**：<br>運行於 Target 社群，接收岸端操作員依序點擊的任意非凸折線（TARGET_PATH_VERTEX），即時轉換為 TGT_WPT_UPDATE 發送給目標船 Helm，支援即時繪製、Undo 與 Clear。 |
| **BHV_AvdColregsV22** | [BHV_AvdColregsV22.cpp](file:///l:/home/yoei/moos-ivp/ivp/src/lib_behaviors-colregs/BHV_AvdColregsV22.cpp) | **國際海上避碰規則（COLREGS）行為**：<br>區分 Rule 14（對遇）、Rule 15（交叉相遇）、Rule 13（追越）、Rule 17（直航船），將對遇判定角度門檻 headon_abs_relbng_thresh 擴大至 .5^\circ$（符合舷燈視角定義），消除對遇時誤轉向左舷的違規風險。 |
| **BHV_Trail** | [BHV_Trail.cpp](file:///l:/home/yoei/moos-ivp/ivp/src/lib_behaviors-marine/BHV_Trail.cpp) | **動態包夾站位行為**：<br>新增 eraseViewableTrailPoint()，在行為轉入 Idle 或被撤銷時，自動清除圖台殘留的灰色航跡點。 |
| **pMarineViewer** | [PMV_GUI.cpp](file:///l:/home/yoei/moos-ivp/ivp/src/pMarineViewer/PMV_GUI.cpp) | **圖台介面核心**：<br>新增 menu_label 設定支援，將岸端按鈕重構為階層式下拉選單（如 Action/Search, Action/COLREGS 等）。 |

---

### 2. ivp/missions/m_shield_demo/ 任務配置與腳本

| 檔案名稱 | 核心角色與配置說明 |
| :--- | :--- |
| [meta_shoreside.moos](file:///l:/home/yoei/moos-ivp/ivp/missions/m_shield_demo/meta_shoreside.moos) | **岸端總控中心**：<br>1. 整合全量按鈕與下拉選單（DEPLOY, RETURN, SURVEY, INTERCEPT, PAT:*, SEE:* 等）。<br>2. 運行 uTimerScript_Heartbeat 每 2 秒廣播一次心跳包。<br>3. 監控全域變數看板（SHIELD_STATE, SHIELD_EVICTIONS）。 |
| [meta_mothership.moos](file:///l:/home/yoei/moos-ivp/ivp/missions/m_shield_demo/meta_mothership.moos) | **母船中樞**：<br>1. 運行 pRegionDivider 與 pTargetCoordinator。<br>2. 設定 pShare 將分區航線與夾擊站位**直接推送**至各子船社群。 |
| [meta_vehicle.moos](file:///l:/home/yoei/moos-ivp/ivp/missions/m_shield_demo/meta_vehicle.moos)<br>[meta_vehicle.bhv](file:///l:/home/yoei/moos-ivp/ivp/missions/m_shield_demo/meta_vehicle.bhv) | **子船 (abe, ben) 配置與行為樹**：<br>1. 5 層模式狀態機：STATION-KEEPING > RETURNING > INTERCEPTING > SURVEYING > LOITERING。<br>2. 配置兩組獨立的 BHV_AvdColregsV22：一組針對隊友（45m 警戒），一組針對目標（22m 警戒，低於 25m 包夾站位，避免自我排斥）。<br>3. 航向 PID 控制器調優： = 0$（消除積分飽和過衝）， = 0.3$（平抑追逐轉向時的舵鋸齒）。 |
| [meta_target.moos](file:///l:/home/yoei/moos-ivp/ivp/missions/m_shield_demo/meta_target.moos)<br>[meta_target.bhv](file:///l:/home/yoei/moos-ivp/ivp/missions/m_shield_demo/meta_target.bhv) | **目標船 (target) 配置與行為樹**：<br>1. 運行 pTargetPathPlanner 支援自訂入侵航線。<br>2. 配置高權重 BHV_AvoidCollision（ = 250$，高於巡邏  = 100$，警戒半徑 55m/25m），受子船雙側逼近時被迫轉向逃離。 |
| [launch.sh](file:///l:/home/yoei/moos-ivp/ivp/missions/m_shield_demo/launch.sh) 等啟動腳本 | **自動化多社群啟動腳本**：一鍵循序啟動 5 個獨立 MOOSDB 與 App 實例（Abe: 9001/9201, Ben: 9002/9202, Target: 9003/9203, Mothership: 9005/9205, Shoreside: 9000/9200）。 |

---

# 二、 MOOS 架構思維：任務狀態、通訊路由與參數哲學

### 1. 母船運算中樞與 pShare 「直送」機制

在標準 MOOS 任務中，跨船通訊通常由 Shoreside 的 uFldShoreBroker 進行橋接（qbridge）。但在本系統中，母船承擔高階運算大腦，通訊流向如下：

`
[操作員介面 Shoreside: 9200]
   │ (MISSION_POLY, REGION_VERTEX, SEARCH_PATTERN, SENSOR_RADIUS)
   ▼
[母船運算中樞 Mothership: 9205]
   │
   ├───【pShare 直送】───► [USV 1 abe: 9201] (WPT_UPDATE, TRAIL_UPDATE, TARGET_ALERT_AUTO)
   └───【pShare 直送】───► [USV 2 ben: 9202] (WPT_UPDATE, TRAIL_UPDATE, TARGET_ALERT_AUTO)
`

> [!IMPORTANT]
> **為什麼母船指令必須以 pShare 直送，不能走 Shoreside 中繼？**
> pShare 內部具備避免死循環的保護機制：**若一個社群對同一個變數名稱同時存在 input 與 output 路由，pShare 會拒絕將收到的網路封包寫入本地 MOOSDB**。Shoreside 的 uFldShoreBroker 已經把 WPT_UPDATE 設定為 qbridge 路由；若母船再把更新丟給 Shoreside，會觸發此機制導致變數被完全丟棄。因此母船以 pShare 直接對準子船的 pShare 端口直灌變數。

---

### 2. 全量狀態發布（Complete Mode State）防止殘留

在 meta_shoreside.moos 與 TargetCoordinator::postAlert() 中，每次狀態轉換均強制發布**完整布林旗標集合**：
`ini
button_three = SURVEY # SURVEY_ALL=true # LOITER_ALL=false
button_three = TARGET_ALERT_ALL=false # RETURN_ALL=false # STATION_KEEP_ALL=false
`
* **設計思維**：若僅送出 SURVEY_ALL=true，先前若處於 INTERCEPTING 狀態，子船 Helm 中的 TARGET_ALERT 仍為 	rue。依行為樹優先權 INTERCEPTING > SURVEYING，子船將永遠卡在攔截模式。全量覆寫確保任意模式切換或重複執行（Re-deploy）均為確定性狀態。

---

### 3. 重送防競態（Repost Mechanism）
在 pRegionDivider 中配置 m_reposts_per_deploy = 3、m_repost_interval = 1.5s：
* **原理**：當點下 DEPLOY 時，子船的 pHelmIvP 剛由 Inactive 進入 Active，各 Behavior 實例尚在初始化。若母船在第 0 毫秒發布 WPT_UPDATE，尚未準備好的 Helm 會直接丟棄此 Mail。間隔 1.5 秒重複補發 3 次，確保非同步分散式進程順利交握。

---

### 4. 全自主接戰與驅逐閉環狀態機（Autonomous Engagement Cycle）

pTargetCoordinator 實現了無須人工干預的完整攻防閉環：

`mermaid
stateDiagram-v2
    [*] --> SEARCHING: DEPLOY
    
    SEARCHING --> INTERCEPTING: 目標進入任一子船感測範圍 (contactHeld = true)<br/>且位於任務區內
    
    state INTERCEPTING {
        [*] --> PincerWall: 評估最佳逃逸方向 (updateExitDirection)
        PincerWall --> Tracking: 左右翼非交叉站位 (bestAssignment)
        Tracking --> Herding: 雙側壓迫迫使目標朝外海轉向
    }
    
    INTERCEPTING --> SEARCHING: 目標脫離區域邊界超過 buffer (targetClearOfRegion)<br/>且持續時間超過 release_hold (8 秒)
    
    note right of SEARCHING
        驅逐計數器 +1 (SHIELD_EVICTIONS)
        進入 reengage_delay (15 秒冷卻防抖)
        自動發布 SURVEY_AUTO=true
    end note
`

* **為什麼不能僅憑「脫離感測範圍（Contact Lost）」判定驅逐成功？**
  若以失去感測為脫離條件，當目標船在任務區內部因避碰微幅轉向而短暫脫離感測圈時，子船會立即解除攔截，目標便能趁機再次深入防區。因此系統要求：**必須同時滿足「目標位於任務區外（含 buffer）」且「持續達 elease_hold 秒（預設 8 秒）」**，才算真正驅逐成功。

---

# 三、 區域切割、分配與掃描規劃原理

pRegionDivider 模組透過嚴謹的計算幾何演算法，將操作員繪製的任意凸多邊形轉化為高效的搜索航線：

`
           [操作員滑鼠標點 REGION_VERTEX]
                        │
                        ▼
            ConvexHullGenerator (凸包計算)
                        │
                        ▼
         Rotating Calipers 旋轉卡尺法
        計算最小外接矩形 (OBB) 之長軸
                        │
         ┌──────────────┴──────────────┐
         ▼                             ▼
   Sweep Angle (主軸方向)        Cross-track (垂直軸均分)
         │                             │
         │                   依子船數 N 等分為 N 條 Band
         │                             │
         └──────────────┬──────────────┘
                        ▼
          在各 Band 內依主軸方向生成航線
         (Lawnmower / Skip / Spiral / Perimeter)
                        │
                        ▼
        感測半徑自適應航寬與端點內縮 (Inset)
`

### 1. 旋轉卡尺主軸對齊（Principal Axis Sweep Alignment）
* **演算法實作 (sweepAngle())**：
  遍歷凸多邊形每一條邊，將所有頂點投影至該邊方向，計算包含所有頂點的最小面積外接矩形（Minimum-Area Bounding Box）。以該矩形的**長邊方向**作為掃描航向（sweep_ang）。
* **效益**：若水域為斜向狹長形狀，傳統固定東西向（East-West）掃描會產生大量極短航線與頻繁的 180° 急轉彎；沿長軸鋪設航線可最大化直線巡航距離，大幅減少調頭次數。

### 2. 跨航向分區切割（Cross-track Band Slicing）
* **演算法實作 (uildPlans())**：
  沿著垂直於長軸的方向（Cross-track），將水域跨向範圍 $[v_{\min}, v_{\max}]$ 均分為 $ 條獨立的 Band（子船  \sim N$ 各分配一條）。
* **為什麼不沿長軸切？**
  若順著長軸切，每艘船分配到的航線長度減半，掉頭次數直接翻倍。跨向切割使每艘船享有全水域長度，整體船隊的轉彎總次數與單船作業完全相同。

### 3. 四大掃描樣式原理（Search Patterns）

`
[1. Lawnmower 割草機]     [2. Skip 跳行模式 (防急轉)]
 ┌───────┐                  ┌───────────────┐
 │ ┌───┐ │                  │ ┌───────────┐ │
 │ │   │ │                  │ │ (2倍航寬迴轉)│
 └─┘   └─┘                  └─┘           └─┘
`

1. **Lawnmower（割草機模式）**：依序掃描相鄰航道。航線最短，但相鄰航道間距小，180° 調頭時容易超調。
2. **Skip（跳行模式，推薦實務使用）**：去程先掃第 , 2, 4$ 軌，回程再掃補第 , 1$ 軌。**調頭轉彎直徑擴大為 2 倍航寬**，徹底解決無人船迴轉半徑受限時的舵面劇烈擺盪問題。
3. **Spiral（向心矩形螺旋）**：由 Band 外圍向中心矩形環狀收縮。
4. **Perimeter（周界巡邏）**：僅沿所屬 Band 的最外圈巡邏。

### 4. 感測半徑自適應與端點內縮（Sensor Spacing & Inset）
在 laneWidth() 與 laneExtent() 中：
\text{Lane Width} = 2 \times R_{\text{sensor}} \times (1 - \text{overlap\_factor})
\text{Endpoint Inset} = 0.70 \times R_{\text{sensor}}
* 當 {\text{sensor}} = 15\text{ m}$、重疊率 \%$ 時，航寬自動設為 \text{ m}$。
* 航線端點向內縮減 .5\text{ m}$，因為船隻到達端點時前向感測器已探測前方水域，無需將船體開至邊界邊緣，大幅節省端點空轉時間。

---

# 四、 驅逐陣型與子船包夾點演算法詳解

驅逐機制的核心並非在目標後方尾隨（Tail-chasing），而是在目標船與受保護水域之間建立一道**動態人字形防衛阻擋牆（Inboard Blocking Arc & Herd Pincer）**。

`
                     任務區中心 / 逃逸反向 (Inboard Base Angle)
                                     ▲
                                     │
                             [阻擋弧 Blocking Arc]
                              abe ───┼─── ben
                               \     │     /
                                \    │    / 左右各 60° (開角 120°)
                                 \   │   /  站位距離 trail_range = 25m
                                  ▼  │  ▼
                               [入侵目標 Target]
                                     │
                                     ▼
                        【受雙側避碰逼迫，朝外海逃逸】
`

### 1. 具體程式碼如何決定子船的下一個包夾點？

包夾點的計算集中在 [TargetCoordinator.cpp](file:///l:/home/yoei/moos-ivp/ivp/src/pTargetCoordinator/TargetCoordinator.cpp)，演算法包含六大步驟：

#### 步驟一：評估最佳逃逸方向 (updateExitDirection & exitCost)
以 ^\circ$ 為間隔，對 36 個可能方向進行射線投影（Ray Casting），計算將目標推出任務區的代價函數：
\text{Cost}(\theta) = \text{DistToBoundary}(x_{\text{tgt}}, y_{\text{tgt}}, \theta) + \text{heading\_bias} \times |\theta - \text{Heading}_{\text{tgt}}|
* 其中 $\text{heading\_bias} = 2.0\text{ m/deg}$。這項權重讓系統**順應目標船當前航向**（若要目標向後掉頭 180°，會額外附加 360m 的代價）。
* 系統選取代價最低的角度作為目標的最佳逃逸方向 $\theta_{\text{exit}}$，並加入 m_exit_margin 滯後門檻，防止目標航向微小抖動時逃逸方向高頻跳變。

#### 步驟二：決定阻擋弧基底角度 (pincerBaseAngle)
阻擋牆必須設在逃逸方向的反方向（Inboard）：
\theta_{\text{base}} = \theta_{\text{exit}} + 180^\circ
子船永遠擋在「目標與受保護區中心」之間。

#### 步驟三：計算扇形站位槽（computeSlots）
將 $ 艘子船均勻分佈在開角 $\alpha = 120^\circ$（spread_deg）的圓弧上：
\text{SlotAngle}_i = \theta_{\text{base}} - \frac{\alpha}{2} + i \times \frac{\alpha}{N-1} \quad (i = 0, \dots, N-1)
以兩艘船為例，站位分別落在 $\theta_{\text{base}} - 60^\circ$ 與 $\theta_{\text{base}} + 60^\circ$。

#### 步驟四：船艏避碰安全防護（owGuard）
檢查每個 Slot 是否落在目標船正前方船艏危險區：
\text{Offset} = |\text{SlotAngle}_i - \text{Heading}_{\text{tgt}}|
若 $\text{Offset} < \text{bow\_guard\_deg}$（預設 ^\circ$），強制將該 Slot 向外側推移至保護角邊界，**嚴禁子船直接停在目標船正前方對衝**。

#### 步驟五：非交叉最佳分配與滯後防抖（estAssignment）
計算每艘子船相對於目標的當前方位角，依照角度排序將左翼子船分配給左側 Slot、右翼分配給右側 Slot。
* **滯後防抖（swap_margin_deg = 30）**：
  \text{Cost}_{\text{current}} - \text{Cost}_{\text{new}} > 30^\circ
  只有當新分配的角度節省大於 ^\circ$ 時才允許交換角色，徹底消除兩船在對稱線附近以 2Hz 高頻互換站位導致的原地打轉現象。

#### 步驟六：站位指令發布（ssignAndPost）
對每艘子船發布絕對方位角追蹤更新：
`ini
TRAIL_UPDATE_ABE = trail_angle=145.2 # trail_angle_type=absolute # trail_range=25.0
`

---

### 2. 驅逐推擠與防衛區域的物理機制

驅逐效果並非靠硬性碰撞，而是透過雙方 **IvP Helm 避碰客觀函數（Objective Function）的權重壓迫**：

| 船隻 | 行為模組 | 權重 (pwt) | 距離設定 | 互動機制 |
| :--- | :--- | :--- | :--- | :--- |
| **目標船 (Target)** | BHV_AvoidCollision | **250**（遠高於巡邏 100） | pwt_outer_dist = 55m<br>pwt_inner_dist = 25m | 當 abe 與 ben 維持在 25m 包夾站位時，進入目標避碰核心區。由於左右兩翼同時存在避碰障礙，目標船唯一零懲罰的航向就是**背離阻擋牆向外逃逸**。 |
| **子船 (Abe, Ben)** | BHV_AvdColregsV22 (目標專用) | **300** | pwt_outer_dist = 22m<br>pwt_inner_dist = 14m | 避碰作用距離（22m）低於包夾站位距離（25m），保證子船在 25m 站位時不會觸發自身避碰而自我排斥。 |

* **航速優勢**：子船最大航速 .0\text{ m/s}$，目標船最大航速 .2\text{ m/s}$（具備 2.5 倍速差），確保子船能在目標轉彎逃竄時持續保持外側包夾包圍圈。

---

# 五、 避碰偵測、COLREGS 合規與面積著色設定

### 1. COLREGS 國際避碰規則實現 (BHV_AvdColregsV22)

為符合國際海事避碰法規，子船採用 BHV_AvdColregsV22 取代單純算 CPA 距離的傳統避碰：
1. **Rule 14 (對遇 Head-on)**：雙方各自向右舷（Starboard）轉向，由左舷（Port-to-port）通過。
   * 修改重點：將對遇判定角度門檻 headon_abs_relbng_thresh 設為 .5^\circ$（符合國際規則舷燈角度定義），防止對遇時誤判為交叉相遇而向左轉。
2. **Rule 15 (交叉相遇 Crossing)**：他船在右舷者為讓路船（Give-way），須及早採取避讓且不得橫越他船船艏；他船在左舷者為直航船（Stand-on）。
3. **Rule 13 (追越 Overtaking)**：追越船須全面保持避讓。
4. **雙軌避碰分離**：
   * vdcol_：針對友船，警戒半徑 45m，安全通過距離 12~20m。
   * vdtgt_：針對目標船，警戒半徑 22m，與包夾站位 25m 形成 3m 安全緩衝帶。

---

### 2. 面積著色與即時覆蓋率網格（XYConvexGrid）

* **網格建立 (initCoverageGrid)**：
  在 pRegionDivider 中，以凸包多邊形為邊界建立 XYConvexGrid，格點尺寸 cell_size = 10m，初始變數 swept = 0。
* **動態覆蓋著色 (handleMailNodeReport)**：
  當收到子船 NODE_REPORT 時，計算船位至各格點中心的距離。凡小於 sensor_radius (15m) 的網格，將 swept 標記為 1，並以 VIEW_GRID_DELTA 廣播至圖台渲染出淡藍色已掃描區塊，直觀呈現尚未覆蓋之盲區。

---

# 六、 全系統通訊與資料傳遞矩陣 (Data Flow Matrix)

`
[ Shoreside ] ──(MISSION_POLY, REGION_VERTEX)──► [ Mothership (pRegionDivider) ]
[ Shoreside ] ──(TARGET_PATH_VERTEX)───────────► [ Target (pTargetPathPlanner) ]
[ Mothership (pRegionDivider) ] ──(REGION_CENTER, REGION_POLY)──► [ Mothership (pTargetCoordinator) ]
[ Mothership (pRegionDivider) ] ──(WPT_UPDATE_*) ───────────────► [ Fleet (Abe, Ben Helm) ]
[ Mothership (pTargetCoordinator) ] ──(TRAIL_UPDATE_*) ────────► [ Fleet (Abe, Ben Helm) ]
[ Mothership ] ──(VIEW_GRID_DELTA, VIEW_SEGLIST)──────────────► [ Shoreside (pMarineViewer) ]
[ Fleet & Target ] ──(NODE_REPORT_LOCAL)───────────────────────► [ All Communities ]
`

| 發布進程 (Source) | 發布變數 (Variable) | 接收進程 (Destination) | 傳輸方式 | 業務功能說明 |
| :--- | :--- | :--- | :--- | :--- |
| pMarineViewer (Shoreside) | REGION_VERTEX / MISSION_POLY | pRegionDivider (Mothership) | pShare (9200 → 9205) | 傳送搜索區域頂點或預設多邊形 |
| pMarineViewer (Shoreside) | SEARCH_PATTERN / SENSOR_RADIUS | pRegionDivider (Mothership) | pShare (9200 → 9205) | 切換航線樣式（Lawn/Skip/Spiral/Perim）與動態調整視距 |
| pMarineViewer (Shoreside) | TARGET_PATH_VERTEX | pTargetPathPlanner (Target) | pShare (9200 → 9203) | 傳送自訂入侵目標航線點序列 |
| pRegionDivider (Mothership) | REGION_CENTER / REGION_POLY | pTargetCoordinator (Mothership) | Local MOOSDB | 提供區域質心與精確邊界多邊形 |
| pRegionDivider (Mothership) | WPT_UPDATE_ABE / _BEN | BHV_Waypoint (Abe / Ben Helm) | **pShare 直送 (9205 → 9201/9202)** | 下發分區搜索割草機/跳行航點序列 |
| pRegionDivider (Mothership) | VIEW_GRID / VIEW_GRID_DELTA | pMarineViewer (Shoreside) | pShare (9205 → 9200) | 回傳面積著色網格與覆蓋率增量更新 |
| pTargetCoordinator (Mothership) | TRAIL_UPDATE_ABE / _BEN | BHV_Trail (Abe / Ben Helm) | **pShare 直送 (9205 → 9201/9202)** | 下發絕對方位角包夾站位指令 |
| pTargetCoordinator (Mothership) | TARGET_ALERT_AUTO / SURVEY_AUTO | pHelmIvP (Abe / Ben Helm) | **pShare 直送 (9205 → 9201/9202)** | 自主驅動狀態機在搜索與攔截間切換 |
| pTargetCoordinator (Mothership) | VIEW_SEGLIST (pincer) / VIEW_POINT | pMarineViewer (Shoreside) | pShare (9205 → 9200) | 繪製紅色的包夾臂線段與預定站位點 |
| pTargetCoordinator (Mothership) | SHIELD_EVICTIONS / SHIELD_STATE | pMarineViewer (Shoreside) | pShare (9205 → 9200) | 於操作台即時顯示累計驅逐成功次數與當前戰術狀態 |
| pNodeReporter (各船隻) | NODE_REPORT_LOCAL | 所有社群 (pContactMgrV20, pTC, pRD, PMV) | pShare / uFldNodeBroker | 廣播即時船位座標、航向與航速 |
