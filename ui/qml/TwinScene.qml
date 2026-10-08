import QtQuick
import QtQuick3D

Item {
    id: root

    function parseState(text) {
        if (!text || text.length === 0) return ({})
        try { return JSON.parse(text) } catch (e) { console.error("Twin state JSON parse failed:", e); return ({}) }
    }
    property string stateText: twinBridge ? twinBridge.stateJson : "{}"
    property var st: parseState(stateText)

    // Keep one persistent delegate per task cargo. Updating roles in-place is
    // important: rebuilding a JS-array Repeater on every telemetry snapshot
    // would destroy the delegates and make carried loads appear to teleport.
    ListModel { id: cargoModel; dynamicRoles: true }
    property int cargoVisualCount: cargoModel.count
    function syncCargoModel() {
        var items = st.cargo_inventory || []
        if (cargoModel.count !== items.length) {
            cargoModel.clear()
            for (var a=0; a<items.length; ++a)             cargoModel.append({
                "cargoId":String(items[a].instance_id || a),
                "xMm":0,"yMm":0,"zMm":0,"yawDeg":0,
                "lengthMm":1200,"widthMm":1000,"heightMm":900,
                "stackLayers":1,
                "cargoStatus":"STAGED","carried":false
            })
        }
        for (var i=0; i<items.length; ++i) {
            var c=items[i] || ({}), p=c.pose || ({})
            cargoModel.setProperty(i,"cargoId",String(c.instance_id || i))
            cargoModel.setProperty(i,"xMm",Number(p.x_mm || 0))
            cargoModel.setProperty(i,"yMm",Number(p.y_mm || 0))
            cargoModel.setProperty(i,"zMm",Number(p.z_mm || 0))
            cargoModel.setProperty(i,"yawDeg",Number(p.yaw_deg || 0))
            // Use plan/measured cargo sizes as-is (mm); only guard against zero/negative.
            cargoModel.setProperty(i,"lengthMm",Math.max(50,Number(c.length_mm || 1200)))
            cargoModel.setProperty(i,"widthMm",Math.max(50,Number(c.width_mm || 1000)))
            cargoModel.setProperty(i,"heightMm",Math.max(50,Number(c.height_mm || 900)))
            cargoModel.setProperty(i,"stackLayers",Number(c.stack_layers || 1) >= 2 ? 2 : 1)
            cargoModel.setProperty(i,"cargoStatus",String(c.status || "STAGED"))
            cargoModel.setProperty(i,"carried",String(c.attached_to || "") === "PICK_ARM")
        }
    }
    onStChanged: syncCargoModel()
    Component.onCompleted: syncCargoModel()

    // WORLD: X right/lateral, Y vehicle-forward/longitudinal, Z upward, mm.
    // Scene: X = WORLD X, Z = WORLD Y, Y = WORLD Z, metre.
    function m(v) { return Number(v || 0) / 1000.0 }
    function worldPos(p) {
        p = p || ({})
        return Qt.vector3d(m(p.x_mm !== undefined ? p.x_mm : p.x),
                           m(p.z_mm !== undefined ? p.z_mm : p.z),
                           m(p.y_mm !== undefined ? p.y_mm : p.y))
    }
    function cubeScaleM(x, y, z) { return Qt.vector3d(Number(x)/100.0, Number(y)/100.0, Number(z)/100.0) }
    function cubeScaleMm(x, y, z) { return cubeScaleM(m(x),m(y),m(z)) }
    function cylScaleM(d, h) { return Qt.vector3d(Number(d)/100.0, Number(h)/100.0, Number(d)/100.0) }
    function sphereScaleM(d) { var s=Number(d)/100.0; return Qt.vector3d(s,s,s) }
    function clamp(v,a,b) { return Math.max(a,Math.min(b,v)) }
    function placementTarget() {
        return st.truck && st.truck.current_target ? st.truck.current_target : ({})
    }
    function sceneBounds() {
        // Tight work-cell frame: truck + gantry rails + cargo/arm only.
        // Avoid padding a full plant floor that leaves the gantry tiny on screen.
        var minX=0, maxX=0, minZ=0, maxZ=0, count=0
        function addMm(x,y) {
            var sx=Number(x || 0)/1000.0, sz=Number(y || 0)/1000.0
            if (count===0) { minX=maxX=sx; minZ=maxZ=sz }
            else { minX=Math.min(minX,sx); maxX=Math.max(maxX,sx); minZ=Math.min(minZ,sz); maxZ=Math.max(maxZ,sz) }
            count++
        }
        var tp=st.truck && st.truck.pose ? st.truck.pose : ({})
        var board=measuredBoardExtent()
        var halfLen=board.valid ? Math.max(2500,board.length*0.5) : 6800
        var halfWid=board.valid ? Math.max(900,board.width*0.5) : 1500
        addMm(Number(tp.x_mm||0)-halfWid-500,Number(tp.y_mm||0)-halfLen-600)
        addMm(Number(tp.x_mm||0)+halfWid+500,Number(tp.y_mm||0)+halfLen+600)
        // Dual rails (±3.5 m) with a small margin — not a 8 m-wide empty apron.
        addMm(-3600,railTailY())
        addMm( 3600,railHeadY())
        var corners=cornerArray()
        for (var i=0;i<corners.length;++i) addMm(corners[i].x,corners[i].y)
        if (st.truck && st.truck.regions) {
            for (var r=0;r<st.truck.regions.length;++r) {
                var c=st.truck.regions[r].center_world_xyz_mm || st.truck.regions[r].center_world
                if (c && c.length>=2) addMm(c[0],c[1])
            }
        }
        if (st.cargo_inventory) {
            for (var ci=0;ci<st.cargo_inventory.length;++ci) {
                var cp=(st.cargo_inventory[ci] || ({})).pose || ({})
                addMm(Number(cp.x_mm||0)-700,Number(cp.y_mm||0)-700)
                addMm(Number(cp.x_mm||0)+700,Number(cp.y_mm||0)+700)
            }
        }
        var target=placementTarget()
        if (target.final_world_pose) addMm(target.final_world_pose.x_mm,target.final_world_pose.y_mm)
        if (st.devices) {
            for (var k in st.devices) {
                var d=st.devices[k]
                if (d.kind==="robot" || d.kind==="radar") {
                    var dp=d.pose || ({})
                    addMm(dp.x_mm,dp.y_mm)
                }
            }
        }
        var pad=0.6
        return {x:(minX+maxX)/2, z:(minZ+maxZ)/2,
                spanX:Math.max(7,maxX-minX)+pad, spanZ:Math.max(9,maxZ-minZ)+pad}
    }
    function targetCode() {
        var t=placementTarget()
        return String(t.blind_code || t.region_id || "等待空间规划")
    }
    function targetXyzText() {
        var p=placementTarget().final_world_pose
        if (!p) return "目标坐标尚未生成"
        return "X "+Number(p.x_mm||0).toFixed(0)+"  ·  Y "+Number(p.y_mm||0).toFixed(0)+"  ·  Z "+Number(p.z_mm||0).toFixed(0)+" mm"
    }

    function device(id) { return st.devices && st.devices[id] ? st.devices[id] : ({}) }
    function poseOf(id) { return device(id).pose || ({}) }
    function robotIds() {
        var out=[]
        if (!st.devices) return out
        for (var k in st.devices) if (st.devices[k].kind === "robot") out.push(k)
        out.sort()
        return out
    }
    function activeTasks() {
        var out=[]
        if (!st.devices) return "设备待命"
        for (var k in st.devices) {
            var task=String(st.devices[k].task || "IDLE")
            if (task !== "IDLE" && task !== "CONNECTED" && task !== "ACK") out.push(k+" · "+task)
        }
        return out.length ? out.join("   |   ") : "设备待命"
    }
    function cameraForRobot(robotId) {
        if (!st.cameras) return ({})
        for (var k in st.cameras) if (st.cameras[k].parent_robot_id === robotId) return st.cameras[k]
        return ({})
    }
    function cameraIdForRobot(robotId) {
        if (!st.cameras) return ""
        for (var k in st.cameras) if (st.cameras[k].parent_robot_id === robotId) return k
        return ""
    }
    function localMountPos(cam) {
        var p=cam && cam.mount_pose ? cam.mount_pose : ({})
        return Qt.vector3d(m(p.x_mm),m(p.z_mm),m(p.y_mm))
    }
    function hasCargo() { return st.cargo && (st.cargo.instance_id || st.cargo.cargo_code || st.cargo.cargo_name) }
    function cargoInventorySummary() {
        var items=st.cargo_inventory || [], placed=0, staged=0
        for (var i=0;i<items.length;++i) {
            var status=String((items[i] || ({})).status || "")
            if (status === "PLACED") placed++
            else if (status === "STAGED" || status === "CURRENT") staged++
        }
        return "任务货物 "+items.length+" · 待装 "+staged+" · 已装 "+placed
    }
    function railHeadY() {
        // Clip longitudinal rails to the active work tip (truck / corners / arm).
        var tp=st.truck && st.truck.pose ? st.truck.pose : ({})
        var board=measuredBoardExtent()
        var half=board.valid ? Math.max(2500,board.length*0.5) : 6800
        var maxY=Number(tp.y_mm||9000)+half+500
        var corners=cornerArray()
        for (var i=0;i<corners.length;++i) maxY=Math.max(maxY,corners[i].y+400)
        if (st.devices) {
            for (var k in st.devices) {
                if (st.devices[k].kind!=="robot") continue
                var p=(st.devices[k].pose||({}))
                maxY=Math.max(maxY,Number(p.y_mm||0)+800)
            }
        }
        return maxY
    }
    function railTailY() {
        var minY=600, items=st.cargo_inventory || []
        for (var i=0;i<items.length;++i) {
            var p=(items[i] || ({})).pose || ({})
            minY=Math.min(minY,Number(p.y_mm || 0)-900)
        }
        if (st.devices) {
            for (var k in st.devices) {
                if (st.devices[k].kind!=="robot") continue
                var rp=(st.devices[k].pose||({}))
                minY=Math.min(minY,Number(rp.y_mm||0)-700)
            }
        }
        var tp=st.truck && st.truck.pose ? st.truck.pose : ({})
        var board=measuredBoardExtent()
        var half=board.valid ? Math.max(2500,board.length*0.5) : 6800
        minY=Math.min(minY,Number(tp.y_mm||9000)-half-400)
        return minY
    }
    function cornerArray() {
        var out=[]
        if (!st.truck || !st.truck.corners) return out
        for (var k in st.truck.corners) {
            var p=st.truck.corners[k] || ({})
            out.push({id:k,
                      x:Number(p.x !== undefined ? p.x : p.x_mm || 0),
                      y:Number(p.y !== undefined ? p.y : p.y_mm || 0),
                      z:Number(p.z !== undefined ? p.z : p.z_mm || 0)})
        }
        out.sort(function(a,b){ return Number(a.id.substring(1))-Number(b.id.substring(1)) })
        return out
    }
    function cornerPoint(id) {
        if (!st.truck || !st.truck.corners || !st.truck.corners[id]) return null
        var p=st.truck.corners[id]
        return {x:Number(p.x !== undefined ? p.x : p.x_mm || 0),
                y:Number(p.y !== undefined ? p.y : p.y_mm || 0),
                z:Number(p.z !== undefined ? p.z : p.z_mm || 0)}
    }
    function boardSurface(ids) {
        var pts=[]
        for (var i=0;i<ids.length;++i) { var p=cornerPoint(ids[i]); if(!p) return ({valid:false}); pts.push(p) }
        return extentFromPoints(pts)
    }
    function extentFromPoints(pts) {
        if (!pts || pts.length < 2) return ({valid:false,cx:0,cy:0,cz:0,length:13300,width:2780})
        var minX=pts[0].x,maxX=pts[0].x,minY=pts[0].y,maxY=pts[0].y,z=0
        for (var j=0;j<pts.length;++j) {
            minX=Math.min(minX,pts[j].x); maxX=Math.max(maxX,pts[j].x)
            minY=Math.min(minY,pts[j].y); maxY=Math.max(maxY,pts[j].y); z+=Number(pts[j].z || 0)
        }
        z/=pts.length
        return {valid:true,cx:(minX+maxX)/2,cy:(minY+maxY)/2,cz:z,length:Math.max(50,maxY-minY),width:Math.max(50,maxX-minX)}
    }
    // Default flatbed when no board measurement yet (metres).
    property real authoredDeckLengthM: 13.3
    property real authoredDeckWidthM: 2.78
    function truckSpecExtent() {
        // Prefer explicit board specs written after step-1 measurement (mm).
        var t = st.truck || ({})
        var len = Number(t.length_mm || 0)
        var wid = Number(t.width_mm || 0)
        if (!(len > 50 && wid > 50)) return ({valid:false})
        var pose = t.pose || ({})
        var cz = Number(t.deck_height_mm || pose.z_mm || 0)
        return {
            valid:true,
            cx:Number(pose.x_mm || 0),
            cy:Number(pose.y_mm || 0),
            cz:cz,
            length:len,
            width:wid,
            fromSpecs:true
        }
    }
    function measuredBoardExtent() {
        var s=boardSurface(["P1","P2","P3","P4"])
        if (s.valid) return s
        var s6=boardSurface(["P1","P2","P3","P4","P5","P6"])
        if (s6.valid) return s6
        var arr=cornerArray(), pts=[]
        for (var i=0;i<arr.length;++i) pts.push(arr[i])
        var fromCorners=extentFromPoints(pts)
        if (fromCorners.valid) return fromCorners
        return truckSpecExtent()
    }
    function regionExtentMm(r) {
        r = r || ({})
        var corners = r.corners_world_xyz_mm || r.corners || []
        var pts=[]
        for (var i=0;i<corners.length;++i) {
            var c=corners[i]
            if (!c) continue
            if (Object.prototype.toString.call(c) === "[object Array]" || (typeof c.length === "number" && typeof c !== "string")) {
                if (c.length >= 2) pts.push({x:Number(c[0]||0), y:Number(c[1]||0), z:Number(c[2]||0)})
            } else {
                pts.push({
                    x:Number(c.x !== undefined ? c.x : c.x_mm || 0),
                    y:Number(c.y !== undefined ? c.y : c.y_mm || 0),
                    z:Number(c.z !== undefined ? c.z : c.z_mm || 0)
                })
            }
        }
        if (pts.length >= 2) return extentFromPoints(pts)
        var len=Math.max(50,Number(r.row_length_mm || 1200))
        var board=measuredBoardExtent()
        var width=board.valid ? Math.max(50,board.width/2.0) : 1080
        return {valid:true,cx:0,cy:0,cz:0,length:len,width:width}
    }
    // Re-evaluate when twin state changes (corners / board geometry / truck specs).
    property var boardExtent: {
        var _dep = st
        var fromCorners = measuredBoardExtent()
        if (fromCorners && fromCorners.valid) return fromCorners
        return truckSpecExtent()
    }
    // True deck size in metres — parametric truck is built from these, not stretched.
    property real deckLengthM: {
        var _dep = st
        var ext = boardExtent
        if (ext && ext.valid) return Math.max(2.0, ext.length / 1000.0)
        return authoredDeckLengthM
    }
    property real deckWidthM: {
        var _dep = st
        var ext = boardExtent
        if (ext && ext.valid) return Math.max(1.2, ext.width / 1000.0)
        return authoredDeckWidthM
    }
    property real deckTopY: 1.28
    property real sideRailH: 0.78
    property real deckScaleX: Math.max(0.15, deckLengthM / authoredDeckLengthM)
    property real deckScaleZ: Math.max(0.15, deckWidthM / authoredDeckWidthM)
    function regionColor(r) {
        var tid=st.truck && st.truck.current_target ? st.truck.current_target.region_id : ""
        var rid=r.region_id || r.blind_code || r.id || ""
        if (rid === tid) return "#f9a825"
        if (r.status === "OCCUPIED") return "#78909c"
        if (r.status === "RESERVED") return "#7e57c2"
        return "#43a047"
    }
    function robotColor(id) {
        if (id === "PICK_ARM") return "#1565c0"
        return "#607d8b"
    }

    property var truckPose: st.truck && st.truck.pose ? st.truck.pose : ({})
    property real truckSceneX: m(truckPose.x_mm || 0)
    property real truckSceneZ: m(truckPose.y_mm || 0)
    property var visibleBounds: sceneBounds()
    property real sceneCenterX: visibleBounds.x
    property real sceneCenterZ: visibleBounds.z
    property real sceneCenterY: 1.85
    // Closer auto-frame so gantry/truck fill the view instead of a tiny island.
    property real fitDistance: clamp(Math.max(visibleBounds.spanZ*1.05,visibleBounds.spanX*1.45,12),11,30)
    property real floorSpanX: Math.max(9, visibleBounds.spanX + 2.2)
    property real floorSpanZ: Math.max(11, visibleBounds.spanZ + 2.2)

    // Orbit camera state. Mouse drag rotates; wheel zooms.
    property real orbitYaw: 28
    property real orbitPitch: -40
    property real orbitDistance: fitDistance
    property string viewName: "全景"
    property real dragX: 0
    property real dragY: 0
    property bool orbitDragging: false
    property real dragYawSensitivity: 0.38
    property real dragPitchSensitivity: 0.30
    property real wheelZoomSensitivity: 0.028

    function setView(name,yaw,pitch,distance) {
        viewName=name
        orbitYaw=yaw
        orbitPitch=pitch
        orbitDistance=distance > 0 ? distance : fitDistance
    }

    Behavior on orbitYaw { enabled: !root.orbitDragging; NumberAnimation { duration: 320; easing.type: Easing.InOutCubic } }
    Behavior on orbitPitch { enabled: !root.orbitDragging; NumberAnimation { duration: 320; easing.type: Easing.InOutCubic } }
    Behavior on orbitDistance { NumberAnimation { duration: 120; easing.type: Easing.OutCubic } }

    Rectangle { anchors.fill: parent; color: "#d5dae2" }

    View3D {
        id: view3d
        anchors.fill: parent
        camera: sceneCamera
        environment: SceneEnvironment {
            clearColor: "#d5dae2"
            backgroundMode: SceneEnvironment.Color
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
        }

        Node {
            id: orbitTarget
            position: Qt.vector3d(root.sceneCenterX, root.sceneCenterY, root.sceneCenterZ)
            Node {
                id: yawRig
                eulerRotation.y: root.orbitYaw
                Node {
                    id: pitchRig
                    eulerRotation.x: root.orbitPitch
                    PerspectiveCamera {
                        id: sceneCamera
                        position: Qt.vector3d(0,0,root.orbitDistance)
                        fieldOfView: 36
                        clipNear: 0.08
                        clipFar: 180
                    }
                }
            }
        }

        // Two soft opposing directional sources give the whole factory a
        // stable level.  The previous local point light created a blown-out
        // hotspot that moved visually with the auto-framed scene centre.
        DirectionalLight { eulerRotation: Qt.vector3d(-48,-28,0); brightness: 0.72; castsShadow: false }
        DirectionalLight { eulerRotation: Qt.vector3d(-38,152,0); brightness: 0.46; castsShadow: false }

        // Plant floor — sized to the work cell (not a huge empty apron).
        Model {
            source: "#Cube"
            position: Qt.vector3d(root.sceneCenterX,-0.055,root.sceneCenterZ)
            scale: root.cubeScaleM(root.floorSpanX,0.11,root.floorSpanZ)
            materials: PrincipledMaterial { baseColor: "#c5cad3"; roughness: 0.96 }
        }
        Repeater3D {
            model: Math.max(5, Math.round(root.floorSpanX/0.9)+1)
            delegate: Model {
                required property int index
                property real step: root.floorSpanX/Math.max(1,Math.round(root.floorSpanX/0.9))
                source: "#Cube"
                position: Qt.vector3d(root.sceneCenterX-root.floorSpanX*0.5+index*step,0.008,root.sceneCenterZ)
                scale: root.cubeScaleM(0.012,0.012,root.floorSpanZ*0.92)
                materials: PrincipledMaterial { baseColor: "#9aa3b2"; opacity: 0.32 }
            }
        }
        Repeater3D {
            model: Math.max(5, Math.round(root.floorSpanZ/2.4)+1)
            delegate: Model {
                required property int index
                property real step: root.floorSpanZ/Math.max(1,Math.round(root.floorSpanZ/2.4))
                source: "#Cube"
                position: Qt.vector3d(root.sceneCenterX,0.009,root.sceneCenterZ-root.floorSpanZ*0.5+index*step)
                scale: root.cubeScaleM(root.floorSpanX*0.92,0.012,0.012)
                materials: PrincipledMaterial { baseColor: "#9aa3b2"; opacity: 0.28 }
            }
        }

        // Dual-side gantry — steel blue-gray (not toy yellow).
        Node {
            id: gantryFrame
            property real leftX: -3.5
            property real rightX: 3.5
            property real tailZ: root.m(root.railTailY())
            property real headZ: root.m(root.railHeadY())
            property real centerZ: (tailZ+headZ)/2
            property real railLength: headZ-tailZ
            property real topY: 5.3
            property color frameColor: "#607d8b"
            property color frameDark: "#455a64"

            // Two fixed longitudinal travel rails.
            Model { source:"#Cube"; position:Qt.vector3d(gantryFrame.leftX,gantryFrame.topY,gantryFrame.centerZ); scale:root.cubeScaleM(0.34,0.34,gantryFrame.railLength); materials:PrincipledMaterial{baseColor:gantryFrame.frameColor;metalness:0.28;roughness:0.43} }
            Model { source:"#Cube"; position:Qt.vector3d(gantryFrame.rightX,gantryFrame.topY,gantryFrame.centerZ); scale:root.cubeScaleM(0.34,0.34,gantryFrame.railLength); materials:PrincipledMaterial{baseColor:gantryFrame.frameColor;metalness:0.28;roughness:0.43} }

            // Four portal support rows keep the long frame visually stable.
            Repeater3D {
                model: 4
                delegate: Node {
                    required property int index
                    property real supportZ: gantryFrame.tailZ + index*gantryFrame.railLength/3
                    Model { source:"#Cube"; position:Qt.vector3d(gantryFrame.leftX,gantryFrame.topY/2,supportZ); scale:root.cubeScaleM(0.34,gantryFrame.topY,0.34); materials:PrincipledMaterial{baseColor:gantryFrame.frameColor;roughness:0.48} }
                    Model { source:"#Cube"; position:Qt.vector3d(gantryFrame.rightX,gantryFrame.topY/2,supportZ); scale:root.cubeScaleM(0.34,gantryFrame.topY,0.34); materials:PrincipledMaterial{baseColor:gantryFrame.frameColor;roughness:0.48} }
                    Model { source:"#Cube"; position:Qt.vector3d(0,gantryFrame.topY+0.30,supportZ); scale:root.cubeScaleM(7.35,0.26,0.26); materials:PrincipledMaterial{baseColor:gantryFrame.frameDark;metalness:0.24;roughness:0.48} }
                    Model { source:"#Cube"; position:Qt.vector3d(gantryFrame.leftX,0.08,supportZ); scale:root.cubeScaleM(0.68,0.10,0.68); materials:PrincipledMaterial{baseColor:gantryFrame.frameDark;metalness:0.20;roughness:0.65} }
                    Model { source:"#Cube"; position:Qt.vector3d(gantryFrame.rightX,0.08,supportZ); scale:root.cubeScaleM(0.68,0.10,0.68); materials:PrincipledMaterial{baseColor:gantryFrame.frameDark;metalness:0.20;roughness:0.65} }
                    // Short diagonal knee braces echo the reference structure.
                    Model { source:"#Cube"; position:Qt.vector3d(gantryFrame.leftX+0.43,gantryFrame.topY+0.02,supportZ); eulerRotation.z:-42; scale:root.cubeScaleM(1.20,0.12,0.12); materials:PrincipledMaterial{baseColor:gantryFrame.frameDark;roughness:0.50} }
                    Model { source:"#Cube"; position:Qt.vector3d(gantryFrame.rightX-0.43,gantryFrame.topY+0.02,supportZ); eulerRotation.z:42; scale:root.cubeScaleM(1.20,0.12,0.12); materials:PrincipledMaterial{baseColor:gantryFrame.frameDark;roughness:0.50} }
                }
            }
        }

        // Parametric flatbed truck: deck built at true measured L×W (metres), cab unstretched.
        Node {
            id: truckNode
            property var tp: root.truckPose
            property bool hasBoard: !!(root.boardExtent && root.boardExtent.valid)
            property real deckL: root.deckLengthM
            property real deckW: root.deckWidthM
            property real deckY: root.deckTopY
            property real railY: root.deckTopY + root.sideRailH * 0.5
            property real halfL: deckL * 0.5
            property real halfW: deckW * 0.5
            property real stakeCount: Math.max(4, Math.min(16, Math.round(deckL / 1.05)))
            property real stakePitch: deckL / Math.max(1, stakeCount)
            property var axleXs: [
                -halfL + Math.min(1.1, deckL * 0.12),
                -halfL + deckL * 0.38,
                halfL - deckL * 0.28,
                halfL - Math.min(1.0, deckL * 0.10)
            ]
            position: hasBoard && root.boardExtent && !root.boardExtent.fromSpecs
                     ? Qt.vector3d(root.m(root.boardExtent.cx), root.m(root.boardExtent.cz), root.m(root.boardExtent.cy))
                     : root.worldPos(tp)
            eulerRotation.y: 90 - Number(tp.yaw_deg || 0)

            // Chassis beam (slightly narrower/shorter than deck).
            Model {
                source: "#Cube"
                position: Qt.vector3d(0, 0.58, 0)
                scale: root.cubeScaleM(truckNode.deckL * 0.96, 0.32, Math.max(0.9, truckNode.deckW * 0.78))
                materials: PrincipledMaterial { baseColor: "#3d4650"; metalness: 0.35; roughness: 0.55 }
            }
            // Main deck plate — exact board length × width (always solid; no fade).
            Model {
                source: "#Cube"
                position: Qt.vector3d(0, truckNode.deckY, 0)
                scale: root.cubeScaleM(truckNode.deckL, 0.18, truckNode.deckW)
                materials: PrincipledMaterial {
                    baseColor: "#8b939e"
                    metalness: 0.16
                    roughness: 0.72
                }
            }
            // Deck planks (visual grain along length).
            Repeater3D {
                model: Math.max(3, Math.min(10, Math.round(truckNode.deckW / 0.28)))
                delegate: Model {
                    required property int index
                    property int n: Math.max(3, Math.min(10, Math.round(truckNode.deckW / 0.28)))
                    property real zOff: -truckNode.halfW + truckNode.deckW * ((index + 0.5) / n)
                    source: "#Cube"
                    position: Qt.vector3d(0, truckNode.deckY + 0.03, zOff)
                    scale: root.cubeScaleM(truckNode.deckL * 0.985, 0.05, Math.max(0.08, truckNode.deckW / n - 0.02))
                    materials: PrincipledMaterial {
                        baseColor: index % 2 === 0 ? "#9aa3ad" : "#7f8893"
                        roughness: 0.78
                    }
                }
            }
            // Side rails.
            Model {
                source: "#Cube"
                position: Qt.vector3d(0, truckNode.railY, -truckNode.halfW + 0.04)
                scale: root.cubeScaleM(truckNode.deckL, root.sideRailH, 0.08)
                materials: PrincipledMaterial {
                    baseColor: "#8b5745"; metalness: 0.08; roughness: 0.82
                }
            }
            Model {
                source: "#Cube"
                position: Qt.vector3d(0, truckNode.railY, truckNode.halfW - 0.04)
                scale: root.cubeScaleM(truckNode.deckL, root.sideRailH, 0.08)
                materials: PrincipledMaterial {
                    baseColor: "#8b5745"; metalness: 0.08; roughness: 0.82
                }
            }
            // Tail / head end rails.
            Model {
                source: "#Cube"
                position: Qt.vector3d(-truckNode.halfL + 0.04, truckNode.railY, 0)
                scale: root.cubeScaleM(0.08, root.sideRailH, truckNode.deckW * 0.96)
                materials: PrincipledMaterial {
                    baseColor: "#7a4c3c"; roughness: 0.80
                }
            }
            Model {
                source: "#Cube"
                position: Qt.vector3d(truckNode.halfL - 0.04, truckNode.railY, 0)
                scale: root.cubeScaleM(0.08, root.sideRailH, truckNode.deckW * 0.96)
                materials: PrincipledMaterial {
                    baseColor: "#7a4c3c"; roughness: 0.80
                }
            }
            // Stake posts along both rails.
            Repeater3D {
                model: Math.round(truckNode.stakeCount)
                delegate: Node {
                    required property int index
                    property real xPos: -truckNode.halfL + (index + 0.5) * truckNode.stakePitch
                    Model {
                        source: "#Cube"
                        position: Qt.vector3d(xPos, truckNode.railY, -truckNode.halfW + 0.01)
                        scale: root.cubeScaleM(0.05, root.sideRailH * 0.92, 0.05)
                        materials: PrincipledMaterial {
                            baseColor: "#b2785d"; roughness: 0.78
                        }
                    }
                    Model {
                        source: "#Cube"
                        position: Qt.vector3d(xPos, truckNode.railY, truckNode.halfW - 0.01)
                        scale: root.cubeScaleM(0.05, root.sideRailH * 0.92, 0.05)
                        materials: PrincipledMaterial {
                            baseColor: "#b2785d"; roughness: 0.78
                        }
                    }
                }
            }
            // Axles + wheels — positions follow deck length; track follows width.
            Repeater3D {
                model: 4
                delegate: Node {
                    required property int index
                    property real axleX: truckNode.axleXs[index]
                    property real track: truckNode.halfW * 0.88
                    Model {
                        source: "#Cylinder"
                        position: Qt.vector3d(axleX, 0.48, 0)
                        eulerRotation.x: 90
                        scale: root.cylScaleM(0.12, truckNode.deckW * 0.92)
                        materials: PrincipledMaterial { baseColor: "#2a3036"; metalness: 0.55; roughness: 0.40 }
                    }
                    Model {
                        source: "#Cylinder"
                        position: Qt.vector3d(axleX, 0.48, -track)
                        eulerRotation.x: 90
                        scale: root.cylScaleM(0.92, 0.32)
                        materials: PrincipledMaterial { baseColor: "#121619"; roughness: 0.97 }
                    }
                    Model {
                        source: "#Cylinder"
                        position: Qt.vector3d(axleX, 0.48, track)
                        eulerRotation.x: 90
                        scale: root.cylScaleM(0.92, 0.32)
                        materials: PrincipledMaterial { baseColor: "#121619"; roughness: 0.97 }
                    }
                    Model {
                        source: "#Cylinder"
                        position: Qt.vector3d(axleX, 0.48, -track - 0.16)
                        eulerRotation.x: 90
                        scale: root.cylScaleM(0.36, 0.05)
                        materials: PrincipledMaterial { baseColor: "#b3bac0"; metalness: 0.82; roughness: 0.22 }
                    }
                    Model {
                        source: "#Cylinder"
                        position: Qt.vector3d(axleX, 0.48, track + 0.16)
                        eulerRotation.x: 90
                        scale: root.cylScaleM(0.36, 0.05)
                        materials: PrincipledMaterial { baseColor: "#b3bac0"; metalness: 0.82; roughness: 0.22 }
                    }
                }
            }

            // Cab — fixed size, parked at the head of the measured deck.
            Node {
                id: cabDecor
                position: Qt.vector3d(-truckNode.halfL - 1.55, 0, 0)
                Model { source:"#Cube"; position:Qt.vector3d(0,1.55,0); scale:root.cubeScaleM(2.65,2.75,Math.min(2.58, truckNode.deckW*0.95)); materials:PrincipledMaterial{baseColor:"#c93443";metalness:0.22;roughness:0.40} }
                Model { source:"#Cube"; position:Qt.vector3d(0.12,3.00,0); scale:root.cubeScaleM(2.30,0.22,Math.min(2.45, truckNode.deckW*0.90)); materials:PrincipledMaterial{baseColor:"#b52a38";metalness:0.20;roughness:0.42} }
                Model { source:"#Cube"; position:Qt.vector3d(1.36,2.12,0); scale:root.cubeScaleM(0.035,1.05,Math.min(2.10, truckNode.deckW*0.82)); materials:PrincipledMaterial{baseColor:"#7fc4d8";opacity:0.78;roughness:0.16} }
                Model { source:"#Cube"; position:Qt.vector3d(0.20,2.15,-Math.min(1.30, truckNode.halfW*0.95)); scale:root.cubeScaleM(1.20,0.85,0.03); materials:PrincipledMaterial{baseColor:"#81c4d8";opacity:0.78;roughness:0.16} }
                Model { source:"#Cube"; position:Qt.vector3d(0.20,2.15, Math.min(1.30, truckNode.halfW*0.95)); scale:root.cubeScaleM(1.20,0.85,0.03); materials:PrincipledMaterial{baseColor:"#81c4d8";opacity:0.78;roughness:0.16} }
                Model { source:"#Cube"; position:Qt.vector3d(-1.38,0.66,0); scale:root.cubeScaleM(0.18,0.28,Math.min(2.52, truckNode.deckW*0.92)); materials:PrincipledMaterial{baseColor:"#c8ccd0";metalness:0.72;roughness:0.28} }
                Model { source:"#Cube"; position:Qt.vector3d(-1.35,1.22,0); scale:root.cubeScaleM(0.05,0.58,1.20); materials:PrincipledMaterial{baseColor:"#24292e";metalness:0.30;roughness:0.52} }
                Model { source:"#Sphere"; position:Qt.vector3d(-1.40,1.18,-0.92); scale:root.sphereScaleM(0.20); materials:PrincipledMaterial{baseColor:"#ffe7a8";emissiveFactor:Qt.vector3d(0.55,0.38,0.12)} }
                Model { source:"#Sphere"; position:Qt.vector3d(-1.40,1.18, 0.92); scale:root.sphereScaleM(0.20); materials:PrincipledMaterial{baseColor:"#ffe7a8";emissiveFactor:Qt.vector3d(0.55,0.38,0.12)} }
                Model { source:"#Cube"; position:Qt.vector3d(0.45,2.35,-Math.min(1.55, truckNode.halfW+0.25)); scale:root.cubeScaleM(0.28,0.42,0.12); materials:PrincipledMaterial{baseColor:"#20252a";metalness:0.25} }
                Model { source:"#Cube"; position:Qt.vector3d(0.45,2.35, Math.min(1.55, truckNode.halfW+0.25)); scale:root.cubeScaleM(0.28,0.42,0.12); materials:PrincipledMaterial{baseColor:"#20252a";metalness:0.25} }
                // Front bumper wheels under cab.
                Model { source:"#Cylinder"; position:Qt.vector3d(0.35,0.48,-1.05); eulerRotation.x:90; scale:root.cylScaleM(0.88,0.30); materials:PrincipledMaterial{baseColor:"#121619";roughness:0.97} }
                Model { source:"#Cylinder"; position:Qt.vector3d(0.35,0.48, 1.05); eulerRotation.x:90; scale:root.cylScaleM(0.88,0.30); materials:PrincipledMaterial{baseColor:"#121619";roughness:0.97} }
            }
        }

        // One transverse beam follows the arm along vehicle Y.  The arm's X
        // position is the trolley coordinate, so it can lower from either rail.
        Repeater3D {
            model: root.robotIds()
            delegate: Node {
                id: movingCrossbeam
                required property string modelData
                property var rp: root.poseOf(modelData)
                position: Qt.vector3d(0,5.08,root.m(rp.y_mm))
                Behavior on z { NumberAnimation{duration:420;easing.type:Easing.InOutCubic} }
                Model { source:"#Cube"; scale:root.cubeScaleM(7.45,0.48,0.58); materials:PrincipledMaterial{baseColor:"#78909c";metalness:0.30;roughness:0.42} }
                Model { source:"#Cube"; position:Qt.vector3d(-3.5,0.10,0); scale:root.cubeScaleM(0.72,0.30,0.82); materials:PrincipledMaterial{baseColor:"#546e7a";metalness:0.35;roughness:0.40} }
                Model { source:"#Cube"; position:Qt.vector3d( 3.5,0.10,0); scale:root.cubeScaleM(0.72,0.30,0.82); materials:PrincipledMaterial{baseColor:"#546e7a";metalness:0.35;roughness:0.40} }
            }
        }

        // The single loading arm remains suspended from the moving crossbeam.
        Repeater3D {
            model: root.robotIds()
            delegate: Node {
                id: robotNode
                required property string modelData
                property var rp: root.poseOf(modelData)
                property var mountedCamera: root.cameraForRobot(modelData)
                property string mountedCameraId: root.cameraIdForRobot(modelData)
                property real toolY: root.m(rp.z_mm)
                property real gantryTopY: 5.1
                property real mastH: Math.max(0.65,gantryTopY-toolY)
                // Both capture yaws are chosen so local +Z points from the
                // mast toward the carried pallet.  Flipping this by WORLD X
                // put the right-side forks on the opposite side of the load.
                property real sideSign: 1
                position: root.worldPos(rp)
                eulerRotation.y: -Number(rp.yaw_deg || 0)

                Behavior on x { NumberAnimation{duration:420;easing.type:Easing.InOutCubic} }
                Behavior on y { NumberAnimation{duration:420;easing.type:Easing.InOutCubic} }
                Behavior on z { NumberAnimation{duration:420;easing.type:Easing.InOutCubic} }

                // trolley sliding left/right on the moving crossbeam
                Model { source:"#Cube"; position:Qt.vector3d(0,robotNode.mastH,0); scale:root.cubeScaleM(1.05,0.42,0.92); materials:PrincipledMaterial{baseColor:root.robotColor(modelData);metalness:0.24;roughness:0.46} }
                // four-frame vertical mast
                Model { source:"#Cube"; position:Qt.vector3d(-0.30,robotNode.mastH/2, -0.25); scale:root.cubeScaleM(0.11,robotNode.mastH,0.11); materials:PrincipledMaterial{baseColor:root.robotColor(modelData)} }
                Model { source:"#Cube"; position:Qt.vector3d( 0.30,robotNode.mastH/2, -0.25); scale:root.cubeScaleM(0.11,robotNode.mastH,0.11); materials:PrincipledMaterial{baseColor:root.robotColor(modelData)} }
                Model { source:"#Cube"; position:Qt.vector3d(-0.30,robotNode.mastH/2,  0.25); scale:root.cubeScaleM(0.11,robotNode.mastH,0.11); materials:PrincipledMaterial{baseColor:root.robotColor(modelData)} }
                Model { source:"#Cube"; position:Qt.vector3d( 0.30,robotNode.mastH/2,  0.25); scale:root.cubeScaleM(0.11,robotNode.mastH,0.11); materials:PrincipledMaterial{baseColor:root.robotColor(modelData)} }
                // lower tool frame
                Model { source:"#Cube"; scale:root.cubeScaleM(0.82,0.32,0.68); materials:PrincipledMaterial{baseColor:root.robotColor(modelData);metalness:0.18;roughness:0.48} }
                // inward telescopic boom -- clearly shows the side-to-inside motion
                Model { source:"#Cube"; position:Qt.vector3d(0,-0.04,0.50*robotNode.sideSign); scale:root.cubeScaleM(0.48,0.18,1.05); materials:PrincipledMaterial{baseColor:"#d7dadd";metalness:0.36;roughness:0.34} }
                Model { source:"#Cube"; position:Qt.vector3d(0,-0.04,0.96*robotNode.sideSign); scale:root.cubeScaleM(0.34,0.15,0.55); materials:PrincipledMaterial{baseColor:"#606a72";metalness:0.42;roughness:0.32} }

                // PICK arm gets a visible fork pair; others remain camera/inspection heads.
                Node {
                    visible: modelData === "PICK_ARM"
                    Model { source:"#Cube"; position:Qt.vector3d(0.25,-0.34,1.10*robotNode.sideSign); scale:root.cubeScaleM(1.35,0.08,0.09); materials:PrincipledMaterial{baseColor:"#afb6ba";metalness:0.65;roughness:0.27} }
                    Model { source:"#Cube"; position:Qt.vector3d(-0.25,-0.34,1.10*robotNode.sideSign); scale:root.cubeScaleM(1.35,0.08,0.09); materials:PrincipledMaterial{baseColor:"#afb6ba";metalness:0.65;roughness:0.27} }
                }

                // Camera is physically mounted on the robot; local mount pose comes from config.
                Node {
                    id: cameraMount
                    visible: robotNode.mountedCameraId !== ""
                    property var mp: robotNode.mountedCamera && robotNode.mountedCamera.mount_pose ? robotNode.mountedCamera.mount_pose : ({})
                    position: root.localMountPos(robotNode.mountedCamera)
                    eulerRotation.x: Number(mp.roll_deg || 0)
                    eulerRotation.y: -Number(mp.yaw_deg || 0)
                    eulerRotation.z: Number(mp.pitch_deg || 0)
                    // camera body
                    Model { source:"#Cube"; scale:root.cubeScaleM(0.25,0.16,0.30); materials:PrincipledMaterial{baseColor:"#e3b82d";metalness:0.22;roughness:0.32} }
                    // black front plate
                    Model { source:"#Cube"; position:Qt.vector3d(0,0,-0.158); scale:root.cubeScaleM(0.19,0.11,0.018); materials:PrincipledMaterial{baseColor:"#181c20";metalness:0.25;roughness:0.28} }
                    // lens
                    Model { source:"#Cylinder"; position:Qt.vector3d(0,0,-0.19); eulerRotation.x:90; scale:root.cylScaleM(0.075,0.065); materials:PrincipledMaterial{baseColor:"#173d54";metalness:0.18;roughness:0.10} }
                    // optical axis hint
                    Model { source:"#Cube"; position:Qt.vector3d(0,0,-0.54); scale:root.cubeScaleM(0.025,0.025,0.70); materials:PrincipledMaterial{baseColor:"#ffd859";opacity:0.42} }
                }
            }
        }

        // Radar stand.
        Node {
            id: radarNode
            property var rp: root.poseOf("RADAR")
            property real h: Math.max(0.6,root.m(rp.z_mm))
            position: root.worldPos(rp)
            Model { source:"#Cylinder"; position:Qt.vector3d(0,-radarNode.h/2,0); scale:root.cylScaleM(0.12,radarNode.h); materials:PrincipledMaterial{baseColor:"#8f999f";metalness:0.45;roughness:0.40} }
            Model { source:"#Cube"; scale:root.cubeScaleM(0.50,0.28,0.38); materials:PrincipledMaterial{baseColor:"#5c6bc0";metalness:0.22;roughness:0.40} }
            Model { source:"#Cube"; position:Qt.vector3d(0.20,0,0); scale:root.cubeScaleM(0.05,0.20,0.26); materials:PrincipledMaterial{baseColor:"#37474f"} }
        }

        // Complete task inventory. Delegates persist by queue index, so a
        // staged pallet is visibly picked, carried and left on the truck while
        // all other staged/placed pallets remain in the scene.
        Repeater3D {
            model: cargoModel
            delegate: Node {
                id: cargoNode
                required property string cargoId
                required property real xMm
                required property real yMm
                required property real zMm
                required property real yawDeg
                required property real lengthMm
                required property real widthMm
                required property real heightMm
                required property int stackLayers
                required property string cargoStatus
                required property bool carried
                position: Qt.vector3d(root.m(xMm),root.m(zMm),root.m(yMm))
                eulerRotation.y: 90 - yawDeg
                Behavior on x { NumberAnimation{duration:420;easing.type:Easing.InOutCubic} }
                Behavior on y { NumberAnimation{duration:420;easing.type:Easing.InOutCubic} }
                Behavior on z { NumberAnimation{duration:420;easing.type:Easing.InOutCubic} }

                property color woodDark: carried ? "#2f6f86" : (cargoStatus === "PLACED" ? "#3a6b52" : "#6b4a2e")
                property color woodMid: carried ? "#3a8fa8" : (cargoStatus === "PLACED" ? "#4a8666" : "#8b6239")
                property color woodLight: carried ? "#4aa8c0" : (cargoStatus === "PLACED" ? "#5a9a78" : "#a67848")
                property color crateColor: carried ? "#f3a84f" : (cargoStatus === "PLACED" ? "#a8c9af" : "#e8ebe6")
                property color crateEdge: carried ? "#d4893a" : (cargoStatus === "PLACED" ? "#7fa18a" : "#c5cac2")
                property color bandColor: carried ? "#ffe0a0" : (cargoStatus === "PLACED" ? "#d0e0d2" : "#b9c0be")
                // Pallet base ~145 mm; cargo heightMm is the load above the deck.
                property real baseH: 0.145
                property real midDeckH: stackLayers >= 2 ? 0.05 : 0
                property real layerGapMm: stackLayers >= 2 ? 50 : 0
                property real usableH: Math.max(80, heightMm)
                property real layerH: stackLayers >= 2 ? Math.max(80, (usableH - layerGapMm) / 2.0) : usableH
                property real halfL: root.m(lengthMm) * 0.5
                property real halfW: root.m(widthMm) * 0.5
                property real blockW: Math.max(0.08, root.m(widthMm) * 0.12)
                property real topY1: baseH + root.m(layerH)
                property real midY: topY1 + root.m(layerGapMm) * 0.5
                property real topY2: midY + midDeckH + root.m(layerH)

                // --- Wood pallet base (stringers + deck boards + fork openings) ---
                Model { // left stringer
                    source:"#Cube"; position:Qt.vector3d(0,0.055,-halfW+blockW*0.55)
                    scale:root.cubeScaleMm(lengthMm,110,blockW*1000)
                    materials:PrincipledMaterial{baseColor:woodDark;roughness:0.85}
                }
                Model { // center stringer
                    source:"#Cube"; position:Qt.vector3d(0,0.055,0)
                    scale:root.cubeScaleMm(lengthMm,110,blockW*1000)
                    materials:PrincipledMaterial{baseColor:woodDark;roughness:0.85}
                }
                Model { // right stringer
                    source:"#Cube"; position:Qt.vector3d(0,0.055,halfW-blockW*0.55)
                    scale:root.cubeScaleMm(lengthMm,110,blockW*1000)
                    materials:PrincipledMaterial{baseColor:woodDark;roughness:0.85}
                }
                // End blocks (create visible fork pockets).
                Repeater3D {
                    model: 3
                    delegate: Node {
                        required property int index
                        property real zPos: index === 0 ? -cargoNode.halfW+cargoNode.blockW*0.55 : (index === 1 ? 0 : cargoNode.halfW-cargoNode.blockW*0.55)
                        Model {
                            source:"#Cube"
                            position:Qt.vector3d(-cargoNode.halfL+root.m(90),0.05,zPos)
                            scale:root.cubeScaleMm(100,100,cargoNode.blockW*1000)
                            materials:PrincipledMaterial{baseColor:cargoNode.woodMid;roughness:0.82}
                        }
                        Model {
                            source:"#Cube"
                            position:Qt.vector3d(cargoNode.halfL-root.m(90),0.05,zPos)
                            scale:root.cubeScaleMm(100,100,cargoNode.blockW*1000)
                            materials:PrincipledMaterial{baseColor:cargoNode.woodMid;roughness:0.82}
                        }
                    }
                }
                // Top deck boards across width.
                Repeater3D {
                    model: Math.max(5, Math.min(11, Math.round(lengthMm / 140)))
                    delegate: Model {
                        required property int index
                        property int n: Math.max(5, Math.min(11, Math.round(cargoNode.lengthMm / 140)))
                        property real xOff: -cargoNode.halfL + root.m(cargoNode.lengthMm) * ((index + 0.5) / n)
                        source:"#Cube"
                        position:Qt.vector3d(xOff, cargoNode.baseH - 0.02, 0)
                        scale:root.cubeScaleMm(Math.max(60, cargoNode.lengthMm / n - 12), 40, cargoNode.widthMm)
                        materials:PrincipledMaterial{
                            baseColor: index % 2 === 0 ? cargoNode.woodLight : cargoNode.woodMid
                            roughness: 0.80
                        }
                    }
                }
                // Fork entry markers on the near face.
                Model {
                    source:"#Cube"
                    position:Qt.vector3d(-halfL*0.25, 0.05, -halfW - 0.005)
                    scale:root.cubeScaleM(0.22, 0.08, 0.015)
                    materials:PrincipledMaterial{baseColor:"#1f2428";roughness:0.6}
                }
                Model {
                    source:"#Cube"
                    position:Qt.vector3d(halfL*0.25, 0.05, -halfW - 0.005)
                    scale:root.cubeScaleM(0.22, 0.08, 0.015)
                    materials:PrincipledMaterial{baseColor:"#1f2428";roughness:0.6}
                }

                // --- Cargo layer 1 (single-layer = full height) ---
                Model {
                    source:"#Cube"
                    position:Qt.vector3d(0, baseH + root.m(layerH)*0.5, 0)
                    scale:root.cubeScaleMm(lengthMm*0.96, layerH, widthMm*0.96)
                    materials:PrincipledMaterial{baseColor:crateColor;metalness:0.03;roughness:0.58}
                }
                Model { // vertical edge trim
                    source:"#Cube"
                    position:Qt.vector3d(0, baseH + root.m(layerH)*0.5, 0)
                    scale:root.cubeScaleMm(lengthMm*0.98, Math.max(40, layerH*0.08), widthMm*0.98)
                    materials:PrincipledMaterial{baseColor:crateEdge;roughness:0.50}
                }
                Model {
                    source:"#Cube"
                    position:Qt.vector3d(0, baseH + root.m(layerH)*0.18, 0)
                    scale:root.cubeScaleMm(lengthMm+20, 45, widthMm+20)
                    materials:PrincipledMaterial{baseColor:bandColor;metalness:0.06;roughness:0.48}
                }
                Model {
                    source:"#Cube"
                    position:Qt.vector3d(0, baseH + root.m(layerH)*0.82, 0)
                    scale:root.cubeScaleMm(lengthMm+20, 45, widthMm+20)
                    materials:PrincipledMaterial{baseColor:bandColor;metalness:0.06;roughness:0.48}
                }

                // --- Two-layer intermediate deck + layer 2 (物品2) ---
                Model {
                    visible: stackLayers >= 2
                    source:"#Cube"
                    position:Qt.vector3d(0, midY, 0)
                    scale:root.cubeScaleMm(lengthMm+30, 45, widthMm+30)
                    materials:PrincipledMaterial{baseColor:woodMid;roughness:0.78}
                }
                Model {
                    visible: stackLayers >= 2
                    source:"#Cube"
                    position:Qt.vector3d(0, midY + midDeckH + root.m(layerH)*0.5, 0)
                    scale:root.cubeScaleMm(lengthMm*0.96, layerH, widthMm*0.96)
                    materials:PrincipledMaterial{baseColor:crateColor;metalness:0.03;roughness:0.58}
                }
                Model {
                    visible: stackLayers >= 2
                    source:"#Cube"
                    position:Qt.vector3d(0, midY + midDeckH + root.m(layerH)*0.18, 0)
                    scale:root.cubeScaleMm(lengthMm+20, 45, widthMm+20)
                    materials:PrincipledMaterial{baseColor:bandColor;metalness:0.06;roughness:0.48}
                }
                Model {
                    visible: stackLayers >= 2
                    source:"#Cube"
                    position:Qt.vector3d(0, midY + midDeckH + root.m(layerH)*0.82, 0)
                    scale:root.cubeScaleMm(lengthMm+20, 45, widthMm+20)
                    materials:PrincipledMaterial{baseColor:bandColor;metalness:0.06;roughness:0.48}
                }
            }
        }

        // Deck surfaces from measured corners (flat or high/low), 1:1 WORLD mm.
        Node {
            id: highLowVisual
            property bool isHighLow: !!(st.truck && String(st.truck.board_mode || "").toUpperCase().indexOf("HIGH") >= 0)
            property var flatSurface: root.boardSurface(["P1","P2","P3","P4"])
            property var highSurface: root.boardSurface(["P1","P2","P3","P4"])
            property var lowSurface: root.boardSurface(["P3","P4","P5","P6"])
            Model {
                visible: !highLowVisual.isHighLow && !!(highLowVisual.flatSurface && highLowVisual.flatSurface.valid)
                source:"#Cube"
                position:Qt.vector3d(root.m(highLowVisual.flatSurface.cx),root.m(highLowVisual.flatSurface.cz)+0.03,root.m(highLowVisual.flatSurface.cy))
                scale:root.cubeScaleMm(highLowVisual.flatSurface.width,70,highLowVisual.flatSurface.length)
                materials:PrincipledMaterial{baseColor:"#90a4ae";opacity:0.88;roughness:0.70;metalness:0.12}
            }
            Model { visible: highLowVisual.isHighLow && !!(highLowVisual.highSurface && highLowVisual.highSurface.valid); source:"#Cube"; position:Qt.vector3d(root.m(highLowVisual.highSurface.cx),root.m(highLowVisual.highSurface.cz)+0.03,root.m(highLowVisual.highSurface.cy)); scale:root.cubeScaleMm(highLowVisual.highSurface.width,70,highLowVisual.highSurface.length); materials:PrincipledMaterial{baseColor:"#a1887f";opacity:0.85;roughness:0.68} }
            Model { visible: highLowVisual.isHighLow && !!(highLowVisual.lowSurface && highLowVisual.lowSurface.valid); source:"#Cube"; position:Qt.vector3d(root.m(highLowVisual.lowSurface.cx),root.m(highLowVisual.lowSurface.cz)+0.03,root.m(highLowVisual.lowSurface.cy)); scale:root.cubeScaleMm(highLowVisual.lowSurface.width,70,highLowVisual.lowSurface.length); materials:PrincipledMaterial{baseColor:"#78909c";opacity:0.85;roughness:0.68} }
        }

        // Loading regions sized from each region's WORLD corners / row_length_mm.
        Repeater3D {
            model: st.truck && st.truck.regions ? st.truck.regions : []
            delegate: Node {
                required property var modelData
                property var c: modelData.center_world_xyz_mm || modelData.center_world || [0,0,0]
                property var extent: root.regionExtentMm(modelData)
                property real halfW: root.m(extent.width) * 0.5
                Model { source:"#Cube"; position:Qt.vector3d(root.m(c[0]),root.m(c[2])+0.025,root.m(c[1])); scale:root.cubeScaleMm(extent.width,36,extent.length); materials:PrincipledMaterial{baseColor:root.regionColor(modelData);opacity:modelData.status==="OCCUPIED"?0.55:0.32;roughness:0.60} }
                Model { source:"#Cube"; position:Qt.vector3d(root.m(c[0])-halfW,root.m(c[2])+0.05,root.m(c[1])); scale:root.cubeScaleMm(14,14,extent.length); materials:PrincipledMaterial{baseColor:"#37474f";opacity:0.45} }
                Model { source:"#Cube"; position:Qt.vector3d(root.m(c[0])+halfW,root.m(c[2])+0.05,root.m(c[1])); scale:root.cubeScaleMm(14,14,extent.length); materials:PrincipledMaterial{baseColor:"#37474f";opacity:0.45} }
            }
        }

        // 4/6 corner markers.
        Repeater3D {
            model: root.cornerArray()
            delegate: Node {
                required property var modelData
                position:Qt.vector3d(root.m(modelData.x),root.m(modelData.z)+0.10,root.m(modelData.y))
                Model { source:"#Sphere"; scale:root.sphereScaleM(0.12); materials:PrincipledMaterial{baseColor:"#c62828";emissiveFactor:Qt.vector3d(0.12,0.01,0.01)} }
            }
        }

        // Current placement target.
        Node {
            id: targetMarker
            visible: !!(st.truck && st.truck.current_target && st.truck.current_target.final_world_pose)
            property var pp: st.truck && st.truck.current_target ? st.truck.current_target.final_world_pose || ({}) : ({})
            position: root.worldPos(pp)
            property real targetLengthMm: Math.max(600,Number(st.cargo && st.cargo.length_mm || 1200))
            property real targetWidthMm: Math.max(500,Number(st.cargo && st.cargo.width_mm || 1000))
            Model { source:"#Cube"; position:Qt.vector3d(0,0.035,0); scale:root.cubeScaleMm(targetMarker.targetLengthMm,60,targetMarker.targetWidthMm); materials:PrincipledMaterial{baseColor:"#f9a825";opacity:0.38;roughness:0.55} }
            Model { source:"#Cylinder"; position:Qt.vector3d(0,0.16,0); scale:root.cylScaleM(0.36,0.28); materials:PrincipledMaterial{baseColor:"#f9a825";emissiveFactor:Qt.vector3d(0.18,0.12,0.01)} }
        }
    }

    // Mouse orbit/zoom surface.
    MouseArea {
        id: orbitMouse
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton
        hoverEnabled: true
        preventStealing: true
        cursorShape: root.orbitDragging ? Qt.ClosedHandCursor : Qt.OpenHandCursor
        onPressed: function(mouse) {
            if (mouse.button !== Qt.LeftButton) return
            mouse.accepted=true
            root.orbitDragging=true
            root.dragX=mouse.x
            root.dragY=mouse.y
        }
        onPositionChanged: function(mouse) {
            if (root.orbitDragging && (mouse.buttons & Qt.LeftButton)) {
                var dx=mouse.x-root.dragX, dy=mouse.y-root.dragY
                root.orbitYaw += dx*root.dragYawSensitivity
                root.orbitPitch = root.clamp(root.orbitPitch-dy*root.dragPitchSensitivity,-88,28)
                root.dragX=mouse.x; root.dragY=mouse.y; root.viewName="自由视角"
            }
        }
        onReleased: function(mouse) { root.orbitDragging=false; mouse.accepted=true }
        onCanceled: root.orbitDragging=false
        onWheel: function(wheel) {
            var delta=Number(wheel.angleDelta.y)
            if (Math.abs(delta)<1 && wheel.pixelDelta) delta=Number(wheel.pixelDelta.y)*8
            root.orbitDistance=root.clamp(root.orbitDistance-delta*root.wheelZoomSensitivity,7,34)
            root.viewName="自由视角"
            wheel.accepted=true
        }
        onDoubleClicked: root.setView("全景",28,-40,0)
    }

    // Overlays removed: default panorama + mouse orbit/zoom/double-click restore.
}
