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
            for (var a=0; a<items.length; ++a) cargoModel.append({
                "cargoId":String(items[a].instance_id || a),
                "xMm":0,"yMm":0,"zMm":0,"yawDeg":0,
                "lengthMm":1200,"widthMm":1000,"heightMm":900,
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
            cargoModel.setProperty(i,"lengthMm",Math.max(300,Number(c.length_mm || 1200)))
            cargoModel.setProperty(i,"widthMm",Math.max(300,Number(c.width_mm || 1000)))
            cargoModel.setProperty(i,"heightMm",Math.max(200,Number(c.height_mm || 900)))
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
        // Fit the complete truck plus task cargo and moving equipment. Always
        // include the truck extent: tail-staged cargo may otherwise pull the
        // auto-frame away from the vehicle before camera corners exist.
        var minX=0, maxX=0, minZ=0, maxZ=0, count=0
        function addMm(x,y) {
            var sx=Number(x || 0)/1000.0, sz=Number(y || 0)/1000.0
            if (count===0) { minX=maxX=sx; minZ=maxZ=sz }
            else { minX=Math.min(minX,sx); maxX=Math.max(maxX,sx); minZ=Math.min(minZ,sz); maxZ=Math.max(maxZ,sz) }
            count++
        }
        var tp=st.truck && st.truck.pose ? st.truck.pose : ({})
        addMm(Number(tp.x_mm||0)-1800,Number(tp.y_mm||0)-8500)
        addMm(Number(tp.x_mm||0)+1800,Number(tp.y_mm||0)+8500)
        // The complete dual-side gantry must remain visible even while the
        // arm is parked on one rail.
        addMm(-4000,railTailY())
        addMm( 4000,railHeadY())
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
                addMm(cp.x_mm,cp.y_mm)
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
        return {x:(minX+maxX)/2, z:(minZ+maxZ)/2,
                spanX:Math.max(4,maxX-minX), spanZ:Math.max(12,maxZ-minZ)}
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
        return 17500
    }
    function railTailY() {
        var minY=1000, items=st.cargo_inventory || []
        for (var i=0;i<items.length;++i) {
            var p=(items[i] || ({})).pose || ({})
            minY=Math.min(minY,Number(p.y_mm || 0)-1100)
        }
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
        var minX=pts[0].x,maxX=pts[0].x,minY=pts[0].y,maxY=pts[0].y,z=0
        for (var j=0;j<pts.length;++j) {
            minX=Math.min(minX,pts[j].x); maxX=Math.max(maxX,pts[j].x)
            minY=Math.min(minY,pts[j].y); maxY=Math.max(maxY,pts[j].y); z+=pts[j].z
        }
        z/=pts.length
        return {valid:true,cx:(minX+maxX)/2,cy:(minY+maxY)/2,cz:z,length:Math.max(50,maxY-minY),width:Math.max(50,maxX-minX)}
    }
    function regionColor(r) {
        var tid=st.truck && st.truck.current_target ? st.truck.current_target.region_id : ""
        var rid=r.region_id || r.blind_code || r.id || ""
        if (rid === tid) return "#ffc83d"
        if (r.status === "OCCUPIED") return "#54758d"
        if (r.status === "RESERVED") return "#a869c5"
        return "#28a77b"
    }
    function robotColor(id) {
        if (id === "PICK_ARM") return "#d9df19"
        return "#d94c36"
    }

    property var truckPose: st.truck && st.truck.pose ? st.truck.pose : ({})
    property real truckSceneX: m(truckPose.x_mm || 0)
    property real truckSceneZ: m(truckPose.y_mm || 0)
    property var visibleBounds: sceneBounds()
    property real sceneCenterX: visibleBounds.x
    property real sceneCenterZ: visibleBounds.z
    property real sceneCenterY: 1.65
    property real fitDistance: clamp(Math.max(28,visibleBounds.spanZ*2.05,visibleBounds.spanX*2.65),28,52)

    // Orbit camera state. Mouse drag rotates; wheel zooms.
    property real orbitYaw: 30
    property real orbitPitch: -32
    property real orbitDistance: fitDistance
    property string viewName: "全景"
    property real dragX: 0
    property real dragY: 0
    property bool orbitDragging: false
    property real dragYawSensitivity: 0.38
    property real dragPitchSensitivity: 0.30
    property real wheelZoomSensitivity: 0.030

    function setView(name,yaw,pitch,distance) {
        viewName=name
        orbitYaw=yaw
        orbitPitch=pitch
        orbitDistance=distance > 0 ? distance : fitDistance
    }

    Behavior on orbitYaw { enabled: !root.orbitDragging; NumberAnimation { duration: 320; easing.type: Easing.InOutCubic } }
    Behavior on orbitPitch { enabled: !root.orbitDragging; NumberAnimation { duration: 320; easing.type: Easing.InOutCubic } }
    Behavior on orbitDistance { NumberAnimation { duration: 120; easing.type: Easing.OutCubic } }

    Rectangle { anchors.fill: parent; color: "#07111b" }

    View3D {
        id: view3d
        anchors.fill: parent
        camera: sceneCamera
        environment: SceneEnvironment {
            clearColor: "#0a1722"
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
                        fieldOfView: 43
                        clipNear: 0.08
                        clipFar: 250
                    }
                }
            }
        }

        // Two soft opposing directional sources give the whole factory a
        // stable level.  The previous local point light created a blown-out
        // hotspot that moved visually with the auto-framed scene centre.
        DirectionalLight { eulerRotation: Qt.vector3d(-48,-28,0); brightness: 0.72; castsShadow: false }
        DirectionalLight { eulerRotation: Qt.vector3d(-38,152,0); brightness: 0.46; castsShadow: false }

        // Factory floor.
        Model {
            source: "#Cube"
            position: Qt.vector3d(root.sceneCenterX,-0.055,root.sceneCenterZ)
            scale: root.cubeScaleM(30,0.11,24)
            materials: PrincipledMaterial { baseColor: "#26323a"; roughness: 0.95 }
        }
        // Floor longitudinal grid.
        Repeater3D {
            model: 17
            delegate: Model {
                required property int index
                source: "#Cube"
                position: Qt.vector3d(root.sceneCenterX-7+index*0.9,0.008,root.sceneCenterZ)
                scale: root.cubeScaleM(0.012,0.012,22)
                materials: PrincipledMaterial { baseColor: "#52616b"; opacity: 0.25 }
            }
        }
        Repeater3D {
            model: 9
            delegate: Model {
                required property int index
                source: "#Cube"
                position: Qt.vector3d(root.sceneCenterX,0.009,root.sceneCenterZ-10.2+index*2.55)
                scale: root.cubeScaleM(28,0.012,0.012)
                materials: PrincipledMaterial { baseColor: "#52616b"; opacity: 0.22 }
            }
        }

        // Dual-side gantry from the reference layout. The one loading arm is
        // suspended from a moving crossbeam and can work from either side.
        Node {
            id: gantryFrame
            property real leftX: -3.5
            property real rightX: 3.5
            property real tailZ: root.m(root.railTailY())
            property real headZ: root.m(root.railHeadY())
            property real centerZ: (tailZ+headZ)/2
            property real railLength: headZ-tailZ
            property real topY: 5.3
            property color frameColor: "#d9df19"
            property color frameDark: "#aeb515"

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

        // Detailed flatbed truck.
        Node {
            id: truckNode
            property var tp: root.truckPose
            position: root.worldPos(tp)
            eulerRotation.y: 90 - Number(tp.yaw_deg || 0)

            // The reported truck pose is the deck centre.  The detailed mesh
            // was authored from the cab/front origin, so centre it on that pose
            // before applying WORLD yaw.  This keeps it under P1-P4/regions.
            Node {
                id: truckBody
                position: Qt.vector3d(-6.15,0,0)

                // chassis and frame
                Model { source:"#Cube"; position:Qt.vector3d(5.4,0.62,0); scale:root.cubeScaleM(12.6,0.34,2.28); materials:PrincipledMaterial{baseColor:"#34393d";metalness:0.32;roughness:0.58} }
                Model { source:"#Cube"; position:Qt.vector3d(6.15,1.28,0); scale:root.cubeScaleM(13.3,0.16,2.78); materials:PrincipledMaterial{baseColor:"#6c625b";metalness:0.18;roughness:0.70} }
            // deck side boards
            Model { source:"#Cube"; position:Qt.vector3d(6.15,1.72,-1.43); scale:root.cubeScaleM(13.3,0.78,0.10); materials:PrincipledMaterial{baseColor:"#8b5745";metalness:0.10;roughness:0.80} }
            Model { source:"#Cube"; position:Qt.vector3d(6.15,1.72, 1.43); scale:root.cubeScaleM(13.3,0.78,0.10); materials:PrincipledMaterial{baseColor:"#8b5745";metalness:0.10;roughness:0.80} }
            // side panel ribs
            Repeater3D {
                model: 12
                delegate: Model {
                    required property int index
                    source:"#Cube"
                    position:Qt.vector3d(0.45+index*1.12,1.72,-1.49)
                    scale:root.cubeScaleM(0.05,0.70,0.05)
                    materials:PrincipledMaterial{baseColor:"#b2785d";roughness:0.78}
                }
            }

            // cab main body
            Model { source:"#Cube"; position:Qt.vector3d(-1.55,1.55,0); scale:root.cubeScaleM(2.65,2.75,2.58); materials:PrincipledMaterial{baseColor:"#c93443";metalness:0.22;roughness:0.40} }
            Model { source:"#Cube"; position:Qt.vector3d(-1.43,3.00,0); scale:root.cubeScaleM(2.30,0.22,2.45); materials:PrincipledMaterial{baseColor:"#b52a38";metalness:0.20;roughness:0.42} }
            // windshield and side windows
            Model { source:"#Cube"; position:Qt.vector3d(-0.19,2.12,0); scale:root.cubeScaleM(0.035,1.05,2.10); materials:PrincipledMaterial{baseColor:"#7fc4d8";opacity:0.78;roughness:0.16} }
            Model { source:"#Cube"; position:Qt.vector3d(-1.35,2.15,-1.305); scale:root.cubeScaleM(1.20,0.85,0.03); materials:PrincipledMaterial{baseColor:"#81c4d8";opacity:0.78;roughness:0.16} }
            Model { source:"#Cube"; position:Qt.vector3d(-1.35,2.15, 1.305); scale:root.cubeScaleM(1.20,0.85,0.03); materials:PrincipledMaterial{baseColor:"#81c4d8";opacity:0.78;roughness:0.16} }
            // bumper + grille + headlights
            Model { source:"#Cube"; position:Qt.vector3d(-2.93,0.66,0); scale:root.cubeScaleM(0.18,0.28,2.52); materials:PrincipledMaterial{baseColor:"#c8ccd0";metalness:0.72;roughness:0.28} }
            Model { source:"#Cube"; position:Qt.vector3d(-2.90,1.22,0); scale:root.cubeScaleM(0.05,0.58,1.20); materials:PrincipledMaterial{baseColor:"#24292e";metalness:0.30;roughness:0.52} }
            Model { source:"#Sphere"; position:Qt.vector3d(-2.95,1.18,-0.92); scale:root.sphereScaleM(0.20); materials:PrincipledMaterial{baseColor:"#ffe7a8";emissiveFactor:Qt.vector3d(0.55,0.38,0.12)} }
            Model { source:"#Sphere"; position:Qt.vector3d(-2.95,1.18, 0.92); scale:root.sphereScaleM(0.20); materials:PrincipledMaterial{baseColor:"#ffe7a8";emissiveFactor:Qt.vector3d(0.55,0.38,0.12)} }
            // mirrors
            Model { source:"#Cube"; position:Qt.vector3d(-1.10,2.35,-1.55); scale:root.cubeScaleM(0.28,0.42,0.12); materials:PrincipledMaterial{baseColor:"#20252a";metalness:0.25} }
            Model { source:"#Cube"; position:Qt.vector3d(-1.10,2.35, 1.55); scale:root.cubeScaleM(0.28,0.42,0.12); materials:PrincipledMaterial{baseColor:"#20252a";metalness:0.25} }

            // wheels and metallic hubs
            Repeater3D {
                model: [ -1.2, 4.2, 9.7 ]
                delegate: Node {
                    required property real modelData
                    Model { source:"#Cylinder"; position:Qt.vector3d(modelData,0.52,-1.26); eulerRotation.x:90; scale:root.cylScaleM(0.94,0.34); materials:PrincipledMaterial{baseColor:"#121619";roughness:0.97} }
                    Model { source:"#Cylinder"; position:Qt.vector3d(modelData,0.52, 1.26); eulerRotation.x:90; scale:root.cylScaleM(0.94,0.34); materials:PrincipledMaterial{baseColor:"#121619";roughness:0.97} }
                    Model { source:"#Cylinder"; position:Qt.vector3d(modelData,0.52,-1.44); eulerRotation.x:90; scale:root.cylScaleM(0.38,0.05); materials:PrincipledMaterial{baseColor:"#b3bac0";metalness:0.82;roughness:0.22} }
                    Model { source:"#Cylinder"; position:Qt.vector3d(modelData,0.52, 1.44); eulerRotation.x:90; scale:root.cylScaleM(0.38,0.05); materials:PrincipledMaterial{baseColor:"#b3bac0";metalness:0.82;roughness:0.22} }
                }
            }
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
                Model { source:"#Cube"; scale:root.cubeScaleM(7.45,0.48,0.58); materials:PrincipledMaterial{baseColor:"#d9df19";metalness:0.30;roughness:0.40} }
                Model { source:"#Cube"; position:Qt.vector3d(-3.5,0.10,0); scale:root.cubeScaleM(0.72,0.30,0.82); materials:PrincipledMaterial{baseColor:"#aeb515";metalness:0.38;roughness:0.36} }
                Model { source:"#Cube"; position:Qt.vector3d( 3.5,0.10,0); scale:root.cubeScaleM(0.72,0.30,0.82); materials:PrincipledMaterial{baseColor:"#aeb515";metalness:0.38;roughness:0.36} }
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
            Model { source:"#Cube"; scale:root.cubeScaleM(0.50,0.28,0.38); materials:PrincipledMaterial{baseColor:"#7e5cb8";metalness:0.25;roughness:0.34} }
            Model { source:"#Cube"; position:Qt.vector3d(0.20,0,0); scale:root.cubeScaleM(0.05,0.20,0.26); materials:PrincipledMaterial{baseColor:"#241d32"} }
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
                required property string cargoStatus
                required property bool carried
                position: Qt.vector3d(root.m(xMm),root.m(zMm),root.m(yMm))
                eulerRotation.y: 90 - yawDeg
                Behavior on x { NumberAnimation{duration:420;easing.type:Easing.InOutCubic} }
                Behavior on y { NumberAnimation{duration:420;easing.type:Easing.InOutCubic} }
                Behavior on z { NumberAnimation{duration:420;easing.type:Easing.InOutCubic} }

                property color palletColor: carried ? "#32b8d8" : (cargoStatus === "PLACED" ? "#3a916f" : "#4f86ba")
                property color crateColor: carried ? "#f3a84f" : (cargoStatus === "PLACED" ? "#a8c9af" : "#e4e7e5")
                property color bandColor: carried ? "#ffe0a0" : (cargoStatus === "PLACED" ? "#d0e0d2" : "#c9cfce")

                Model { source:"#Cube"; position:Qt.vector3d(0,0.06,-root.m(cargoNode.widthMm)*0.36); scale:root.cubeScaleMm(Math.max(cargoNode.lengthMm,1200),120,120); materials:PrincipledMaterial{baseColor:"#366da1";roughness:0.72} }
                Model { source:"#Cube"; position:Qt.vector3d(0,0.06,0); scale:root.cubeScaleMm(Math.max(cargoNode.lengthMm,1200),120,120); materials:PrincipledMaterial{baseColor:"#366da1";roughness:0.72} }
                Model { source:"#Cube"; position:Qt.vector3d(0,0.06, root.m(cargoNode.widthMm)*0.36); scale:root.cubeScaleMm(Math.max(cargoNode.lengthMm,1200),120,120); materials:PrincipledMaterial{baseColor:"#366da1";roughness:0.72} }
                Model { source:"#Cube"; position:Qt.vector3d(0,0.145,0); scale:root.cubeScaleMm(Math.max(cargoNode.lengthMm,1200),70,Math.max(cargoNode.widthMm,1000)); materials:PrincipledMaterial{baseColor:cargoNode.palletColor;roughness:0.66} }
                Model { source:"#Cube"; position:Qt.vector3d(0,0.18+root.m(cargoNode.heightMm)/2,0); scale:root.cubeScaleMm(cargoNode.lengthMm,cargoNode.heightMm,cargoNode.widthMm); materials:PrincipledMaterial{baseColor:cargoNode.crateColor;metalness:0.04;roughness:0.56} }
                Model { source:"#Cube"; position:Qt.vector3d(0,0.18+root.m(cargoNode.heightMm)*0.18,0); scale:root.cubeScaleMm(cargoNode.lengthMm+30,55,cargoNode.widthMm+30); materials:PrincipledMaterial{baseColor:cargoNode.bandColor;metalness:0.08;roughness:0.52} }
                Model { source:"#Cube"; position:Qt.vector3d(0,0.18+root.m(cargoNode.heightMm)*0.82,0); scale:root.cubeScaleMm(cargoNode.lengthMm+30,55,cargoNode.widthMm+30); materials:PrincipledMaterial{baseColor:cargoNode.bandColor;metalness:0.08;roughness:0.52} }
                Model { source:"#Cube"; position:Qt.vector3d(0.24,0.07,-root.m(cargoNode.widthMm)/2-0.01); scale:root.cubeScaleM(0.23,0.10,0.02); materials:PrincipledMaterial{baseColor:"#252a2d"} }
                Model { source:"#Cube"; position:Qt.vector3d(-0.24,0.07,-root.m(cargoNode.widthMm)/2-0.01); scale:root.cubeScaleM(0.23,0.10,0.02); materials:PrincipledMaterial{baseColor:"#252a2d"} }
            }
        }

        // High/low deck surfaces from 6 corners.
        Node {
            id: highLowVisual
            visible: st.truck && (String(st.truck.board_mode).toUpperCase().indexOf("HIGH") >= 0)
            property var highSurface: root.boardSurface(["P1","P2","P3","P4"])
            property var lowSurface: root.boardSurface(["P3","P4","P5","P6"])
            Model { visible:highLowVisual.highSurface.valid===true; source:"#Cube"; position:Qt.vector3d(root.m(highLowVisual.highSurface.cx),root.m(highLowVisual.highSurface.cz)+0.03,root.m(highLowVisual.highSurface.cy)); scale:root.cubeScaleMm(highLowVisual.highSurface.width,60,highLowVisual.highSurface.length); materials:PrincipledMaterial{baseColor:"#a66b3f";opacity:0.74;roughness:0.58} }
            Model { visible:highLowVisual.lowSurface.valid===true; source:"#Cube"; position:Qt.vector3d(root.m(highLowVisual.lowSurface.cx),root.m(highLowVisual.lowSurface.cz)+0.03,root.m(highLowVisual.lowSurface.cy)); scale:root.cubeScaleMm(highLowVisual.lowSurface.width,60,highLowVisual.lowSurface.length); materials:PrincipledMaterial{baseColor:"#567d8e";opacity:0.74;roughness:0.58} }
        }

        // Loading regions two columns x 1.2m.
        Repeater3D {
            model: st.truck && st.truck.regions ? st.truck.regions : []
            delegate: Node {
                required property var modelData
                property var c: modelData.center_world_xyz_mm || modelData.center_world || [0,0,0]
                property real lenM: 1.20
                Model { source:"#Cube"; position:Qt.vector3d(root.m(c[0]),root.m(c[2])+0.025,root.m(c[1])); scale:root.cubeScaleM(1.08,0.04,1.16); materials:PrincipledMaterial{baseColor:root.regionColor(modelData);opacity:modelData.status==="OCCUPIED"?0.62:0.38;roughness:0.55} }
                // white boundary around each cell, approximated with thin strips
                Model { source:"#Cube"; position:Qt.vector3d(root.m(c[0])-0.54,root.m(c[2])+0.05,root.m(c[1])); scale:root.cubeScaleM(0.018,0.018,1.16); materials:PrincipledMaterial{baseColor:"#d6e3e8";opacity:0.65} }
                Model { source:"#Cube"; position:Qt.vector3d(root.m(c[0])+0.54,root.m(c[2])+0.05,root.m(c[1])); scale:root.cubeScaleM(0.018,0.018,1.16); materials:PrincipledMaterial{baseColor:"#d6e3e8";opacity:0.65} }
            }
        }

        // 4/6 corner markers.
        Repeater3D {
            model: root.cornerArray()
            delegate: Node {
                required property var modelData
                position:Qt.vector3d(root.m(modelData.x),root.m(modelData.z)+0.10,root.m(modelData.y))
                Model { source:"#Sphere"; scale:root.sphereScaleM(0.16); materials:PrincipledMaterial{baseColor:"#ff5664";emissiveFactor:Qt.vector3d(0.30,0.02,0.02)} }
                Model { source:"#Cylinder"; position:Qt.vector3d(0,0.17,0); scale:root.cylScaleM(0.035,0.34); materials:PrincipledMaterial{baseColor:"#ff8a92"} }
            }
        }

        // Current placement target.
        Node {
            visible: st.truck && st.truck.current_target && st.truck.current_target.final_world_pose
            property var pp: st.truck && st.truck.current_target ? st.truck.current_target.final_world_pose || ({}) : ({})
            position: root.worldPos(pp)
            property real targetLengthMm: Math.max(600,Number(st.cargo && st.cargo.length_mm || 1200))
            property real targetWidthMm: Math.max(500,Number(st.cargo && st.cargo.width_mm || 1000))
            Model { source:"#Cube"; position:Qt.vector3d(0,0.035,0); scale:root.cubeScaleMm(targetLengthMm,70,targetWidthMm); materials:PrincipledMaterial{baseColor:"#ffd33d";opacity:0.32;emissiveFactor:Qt.vector3d(0.22,0.14,0.01);roughness:0.48} }
            Model { source:"#Cylinder"; position:Qt.vector3d(0,0.18,0); scale:root.cylScaleM(0.52,0.36); materials:PrincipledMaterial{baseColor:"#ffd33d";emissiveFactor:Qt.vector3d(0.42,0.28,0.01)} }
            Model { source:"#Cylinder"; position:Qt.vector3d(0,0.78,0); scale:root.cylScaleM(0.075,1.20); materials:PrincipledMaterial{baseColor:"#fff0a2";emissiveFactor:Qt.vector3d(0.30,0.22,0.04)} }
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
            root.orbitDistance=root.clamp(root.orbitDistance-delta*root.wheelZoomSensitivity,8,55)
            root.viewName="自由视角"
            wheel.accepted=true
        }
        onDoubleClicked: root.setView("全景",30,-32,0)
    }

    // View selector overlay.
    Rectangle {
        id: viewPanel
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: 10
        width: Math.min(parent.width < 800 ? 360 : Math.max(400,parent.width*0.54),parent.width-20)
        height: 88
        radius: 7
        color: "#d00b1824"
        border.color: "#3a6d87"
        border.width: 1
        Column {
            anchors.fill: parent
            anchors.margins: 8
            spacing: 5
            Row {
                spacing: 5
                Repeater {
                    model: [
                        {n:"全景",y:30,p:-32,d:0},
                        {n:"俯视",y:0,p:-88,d:0},
                        {n:"左侧",y:-90,p:-8,d:0},
                        {n:"右侧",y:90,p:-8,d:0},
                        {n:"车头",y:0,p:-8,d:0},
                        {n:"车尾",y:180,p:-8,d:0}
                    ]
                    delegate: Rectangle {
                        required property var modelData
                        width: (viewPanel.width-66)/6; height: 32; radius: 4
                        color: root.viewName===modelData.n ? "#1673a6" : "#173247"
                        border.color: root.viewName===modelData.n ? "#77d7ff" : "#31556a"
                        Text { anchors.centerIn: parent; text:modelData.n; color:"#e8f5fa"; font.pixelSize:12 }
                        MouseArea { anchors.fill:parent; onClicked:root.setView(modelData.n,modelData.y,modelData.p,modelData.d) }
                    }
                }
            }
            Text { color:"#a9c7d5"; font.pixelSize:11; text:"左键拖动旋转 · 滚轮快速缩放 · 双击恢复全景" }
        }
    }

    Rectangle {
        id: sceneInfoPanel
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 10
        // Keep clear of the right view panel; wrap long axis/process text inside the box.
        width: Math.min(parent.width < 800 ? 300 : 420, Math.max(220, parent.width - viewPanel.width - 36))
        height: sceneInfoColumn.implicitHeight + 18
        radius: 7
        color: "#ce0b1824"
        border.color: "#35667f"
        border.width: 1
        clip: true
        Column {
            id: sceneInfoColumn
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.margins: 9
            spacing: 3
            Text {
                width: parent.width
                color: "#dcecff"
                font.pixelSize: root.width < 800 ? 11 : 13
                wrapMode: Text.WordWrap
                text: "数字孪生 · world/mm → Scene/m"
            }
            Text {
                width: parent.width
                color: "#8ad6ff"
                font.pixelSize: root.width < 800 ? 10 : 12
                wrapMode: Text.WordWrap
                text: "视角：" + root.viewName + "   4/6角点：" + root.cornerArray().length
            }
            Text {
                width: parent.width
                color: "#a9c7d5"
                font.pixelSize: root.width < 800 ? 9 : 11
                wrapMode: Text.WordWrap
                text: "X右 · Y车尾→车头 · Z向上｜双侧龙门架 · 左侧一次拍照 · 按目标物理左右选放货侧"
            }
        }
    }

    Rectangle {
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.margins: 10
        width: Math.min(620,parent.width-20)
        height: 76
        radius: 7
        color: "#e00a1826"
        border.color: "#2f89b7"
        border.width: 1
        Column {
            anchors.fill: parent
            anchors.margins: 9
            spacing: 5
            Text {
                width: parent.width; color:"#6fd7ff"; font.pixelSize:13; font.bold:true
                text:"当前流程 · "+String(st.phase || "IDLE"); elide:Text.ElideRight
            }
            Text {
                width: parent.width; color:"#e2eef5"; font.pixelSize:12
                text:root.activeTasks(); elide:Text.ElideRight
            }
            Text {
                color:root.hasCargo() && st.cargo.attached_to ? "#ffd76f" : "#7fa9be"
                font.pixelSize:10
                text:root.cargoInventorySummary() + (root.hasCargo() ? (" ｜ 当前 "+String(st.cargo.status||"-")+(st.cargo.attached_to ? " 随动 "+st.cargo.attached_to : "")) : "")
            }
        }
    }
}
