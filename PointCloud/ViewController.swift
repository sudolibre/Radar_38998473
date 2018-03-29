//
//  ViewController.swift
//  PointCloud
//
//  Created by Jon Day on 10/27/17.
//  Copyright © 2017 com.metal.preprocessing. All rights reserved.
//

import UIKit
import SceneKit
import ARKit
import Vision

class ViewController: UIViewController, ARSCNViewDelegate, ARSessionDelegate {

    @IBOutlet var sceneView: ARSCNView!
    var videoPreviewLayer: CALayer { return view.layer }
    var lastUpdate = Date()
    var firstPosition: float3? = nil
    var alwaysFirst = false
    var count = 0
    var currentDepth: Float? = nil
    var currentCameraLocation: SCNVector3? = nil
    var shouldRemove: Bool = false
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        count += 1
        guard count > 5 else { return }
        count = 0
//        guard Date() > lastUpdate.addingTimeInterval() else {
//            return
//        }
//        lastUpdate = Date()

        if let scenePoints = frame.rawFeaturePoints?.points {
            currentDepth = getMedianDepthInViewSpace(scenePoints)
        }
        self.analyzePixelBuffer(frame.capturedImage, frame: frame)
    }

    func analyzePixelBuffer(_ pixelBuffer: CVPixelBuffer, frame: ARFrame) {
        var width = CVPixelBufferGetWidth(pixelBuffer)
        var height = CVPixelBufferGetHeight(pixelBuffer)

        var requestOptions: [VNImageOption: Any] = [:]

        if let cameraIntrinsics = CMGetAttachment(pixelBuffer, kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix, nil) {
            requestOptions = [.cameraIntrinsics: cameraIntrinsics]
        }

        let interfaceOrientation = UIApplication.shared.statusBarOrientation
        guard let orientation = CGImagePropertyOrientation(interfaceOrientation: interfaceOrientation) else {
            return
        }

        let imageRequestHandler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer, orientation: orientation, options: requestOptions)

        switch orientation {
        case .left, .leftMirrored, .right, .rightMirrored:
            swap(&width, &height)
        case .up, .upMirrored, .down, .downMirrored:
            break
        }

        let detectRectangleRequest = VNDetectRectanglesRequest(completionHandler: handleRectangleObservations)
        detectRectangleRequest.minimumAspectRatio = 1.45
        detectRectangleRequest.maximumAspectRatio = 1.65
        detectRectangleRequest.maximumObservations = 3
        detectRectangleRequest.minimumConfidence = 0.9

        try? imageRequestHandler.perform([detectRectangleRequest])
    }

//    func handleRectangleObservations(_ request: VNRequest, error: Error?) {
//        guard let observations = request.results?.compactMap({$0 as? VNRectangleObservation}),
//            !observations.isEmpty else {
//                return
//        }
//        observations.forEach {
//            processForVisualDebugger($0)
//            if let depth = currentDepth {
//                addBox(rect: $0.boundingBox, depth: depth)
//            }
//        }
//    }
    func handleRectangleObservations(_ request: VNRequest, error: Error?) {
        guard let observations = request.results?.compactMap({$0 as? VNRectangleObservation}),
            !observations.isEmpty else {
                return
        }
        observations.forEach {
            processForVisualDebugger($0)
            let calculatedDepth = calculateDepth(observation: $0)
            if calculatedDepth > 0 {
                print(calculatedDepth.description + " " + (currentDepth?.description ?? ""))
                addBox(rect: $0.boundingBox, depth: calculatedDepth)
            }
        }
    }

    func calculateDepth(observation: VNRectangleObservation) -> Float {
        let corners = [observation.topLeft, observation.topRight, observation.bottomLeft, observation.bottomRight]
        var distances: [Float] = []
        var medianDistance: Float {
            var distance: Float = distances.reduce(into: 0.0, +=)
            if distances.count > 1 {
                distance /= Float(distances.count)
            }
            return distance
        }

        corners.forEach { (point) in
            let hits = sceneView.hitTest(point, types: .estimatedVerticalPlane)
            if let hit = hits.first {
                let scenePoint = SCNVector3(
                    x: hit.worldTransform.columns.3.x,
                    y: hit.worldTransform.columns.3.y,
                    z: hit.worldTransform.columns.3.z
                )
                let viewPoint = sceneView.projectPoint(scenePoint)
                let distance = viewPoint.z
                distances.append(distance)
            }

//            if let cornerDistance = sceneView.hitTest(point, types: .estimatedVerticalPlane).first?.distance {
//                distances.append(cornerDistance)
//            }
        }

        return medianDistance
    }

    func getMedianDepthInViewSpace(_ sceneSpacePoints: [vector_float3]) -> Float {
        let points: [SCNVector3] = sceneSpacePoints.map {
            let sceneVector = SCNVector3($0)
            return sceneView.projectPoint(sceneVector)
        }
        let sortedPoints = points.sorted { $0.z < $1.z }
        let midIndex = points.count / 2
        let medianDepth = sortedPoints[midIndex].z
        return medianDepth
    }

    func addBox(rect: CGRect, depth: Float) {
        let imageRect = VNImageRectForNormalizedRect(rect, Int(view.frame.width), Int(view.frame.height))
        let corners = [
            CGPoint(x: imageRect.minX, y: imageRect.minY),
            CGPoint(x: imageRect.minX, y: imageRect.maxY),
            CGPoint(x: imageRect.maxX, y: imageRect.minY),
            CGPoint(x: imageRect.maxX, y: imageRect.maxY)
        ]
        let midPoint = CGPoint(x: imageRect.midX, y: imageRect.midY)

        var existingLabels: [SCNBox] = []
        var isSpaceAlreadyOccupied: Bool { return !existingLabels.isEmpty }
        (corners + [midPoint]).forEach({ corner in
            let cornerHits = sceneView.hitTest(corner, options: [SCNHitTestOption.searchMode: 1]) //crashes
            let labelBoxes = cornerHits.compactMap({$0.node.geometry as? SCNBox})
            existingLabels.append(contentsOf: labelBoxes)
        })

        guard !isSpaceAlreadyOccupied else {
            return
        }

        let midPointWithDepth = SCNVector3(Float(midPoint.x), Float(midPoint.y), depth)
        let midPointInSceneSpace = sceneView.unprojectPoint(midPointWithDepth)


        let box = SCNBox(width: 0.1016, height: 0.0508, length: 0.05, chamferRadius: 0)
        box.firstMaterial?.transparency = 0.5
        let color = UIColor.red.cgColor
        box.firstMaterial?.diffuse.contents = color
        let boxNode = SCNNode(geometry: box)
        boxNode.physicsBody = SCNPhysicsBody(type: .kinematic, shape: nil)
        boxNode.physicsBody?.isAffectedByGravity = false
        boxNode.physicsBody?.contactTestBitMask = 1
        if let orientation = sceneView.pointOfView?.orientation {
            boxNode.orientation = orientation
        }
        boxNode.position = midPointInSceneSpace

        DispatchQueue.main.async {
            self.sceneView.scene.rootNode.addChildNode(boxNode)
        }
        DispatchQueue.main.asyncAfter(deadline: .now() + 4.0) {
            if self.shouldRemove { boxNode.removeFromParentNode() }
        }
    }

    private func processForVisualDebugger(_ observation: VNRectangleObservation) {
        DispatchQueue.main.async {
            self.barcodeManager(didDetectLabel: observation.boundingBox, corners: [observation.topLeft, observation.topRight, observation.bottomLeft, observation.bottomRight])
        }
    }

    func barcodeManager(didDetectLabel label: CGRect, corners: [CGPoint]) {
        //clearRects()
        let labelRect = normalizeToImageRect(label)
        let cornerPoints = corners.map(normalizeToImagePoint)
        let cornerRects: [CGRect] = cornerPoints.map {
            let side: CGFloat = 10
            return CGRect(x: $0.x - side / 2, y: $0.y - side / 2, width: side, height: side)
        }
        drawRect(labelRect, color: .cyan)
        cornerRects.forEach({drawRect($0, color: .blue, fillColor: .blue)})
    }

    func normalizeToImagePoint(_ point: CGPoint) -> CGPoint {
        return VNImagePointForNormalizedPoint(point, Int(videoPreviewLayer.bounds.width), Int(videoPreviewLayer.bounds.height))
    }

    func normalizeToImageRect(_ rect: CGRect) -> CGRect {
        return VNImageRectForNormalizedRect(rect, Int(videoPreviewLayer.bounds.width), Int(videoPreviewLayer.bounds.height))
    }

    func clearRects(force: Bool = false) {
        view.layer.sublayers?.forEach({if $0 is CAShapeLayer { $0.removeFromSuperlayer()}})
    }

    func drawRect(_ rect: CGRect, color: UIColor, fillColor: UIColor? = nil) {
        let layer = CAShapeLayer()
        layer.path = CGPath.init(rect: rect, transform: nil)
        layer.strokeColor = color.cgColor
        layer.fillColor = fillColor?.cgColor ?? UIColor.clear.cgColor
        view.layer.addSublayer(layer)
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.5) {
            layer.removeFromSuperlayer()
        }
    }

    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        let touch = touches.first!
        let location = touch.location(in: sceneView)
        let hitResults = sceneView.hitTest(location, options: [SCNHitTestOption.searchMode: 1]).flatMap({$0.node.geometry as? SCNBox})
        if let result = hitResults.first {
            result.firstMaterial?.diffuse.contents = UIColor.green
        }
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Set the view's delegate
        sceneView.scene.physicsWorld.contactDelegate = self
        sceneView.delegate = self
        sceneView.session.delegate = self
        sceneView.debugOptions = [SCNDebugOptions(rawValue: ARSCNDebugOptions.showFeaturePoints.rawValue)]
        //SCNDebugOptions.showPhysicsShapes.rawValue |
        // Show statistics such as fps and timing information
        sceneView.showsStatistics = true

//        DispatchQueue.main.asyncAfter(deadline: .now() + 10) {
//            let unscannedLabels = self.sceneView.scene.rootNode.childNodes.filter({$0.geometry is SCNBox}).filter({($0.geometry?.firstMaterial?.diffuse.contents as? UIColor) == UIColor.red})
//            unscannedLabels.forEach { $0.addChildNode(self.getCone()) }
//        }
        // Create a new scene
        //let scene = SCNScene(named: "art.scnassets/ship.scn")!
        
        // Set the scene to the view
       // sceneView.scene = scene
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Create a session configuration
        let configuration = ARWorldTrackingConfiguration()
        configuration.planeDetection = .vertical
        // Run the view's session
        sceneView.session.run(configuration)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        // Pause the view's session
        sceneView.session.pause()
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Release any cached data, images, etc that aren't in use.
    }

    // MARK: - ARSCNViewDelegate
    var planeCount = 0

    func renderer(_ renderer: SCNSceneRenderer, nodeFor anchor: ARAnchor) -> SCNNode? {
        if let planchor = anchor as? ARPlaneAnchor {
            if planchor.alignment == .vertical {
                print("found a vertical plane, boss!")

                let scenePlaneGeometry = ARSCNPlaneGeometry(device: MTLCreateSystemDefaultDevice()!)
                scenePlaneGeometry?.update(from: planchor.geometry)
                let planeNode = SCNNode(geometry: scenePlaneGeometry)
                planeNode.name = "\(planeCount)"
                planeCount += 1
                planeNode.geometry?.firstMaterial?.diffuse.contents = UIColor.cyan.withAlphaComponent(0.5)

                return planeNode            }
        }
        return nil
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        // Present an error message to the user
        
    }
    
    func sessionWasInterrupted(_ session: ARSession) {
        // Inform the user that the session has been interrupted, for example, by presenting an overlay
        
    }
    
    func sessionInterruptionEnded(_ session: ARSession) {
        // Reset tracking and/or remove existing anchors if consistent tracking is required
        
    }
}

extension CGImagePropertyOrientation {
    init?(interfaceOrientation: UIInterfaceOrientation) {
        switch interfaceOrientation {
        case .portrait:
            self = .rightMirrored
        case .portraitUpsideDown:
            self = .leftMirrored
        case .landscapeRight:
            self = .downMirrored
        case .landscapeLeft:
            self = .upMirrored
        case .unknown:
            return nil
        }
    }
}

extension ViewController: SCNPhysicsContactDelegate {
    func physicsWorld(_ world: SCNPhysicsWorld, didBegin contact: SCNPhysicsContact) {
        //print("contact detected")
        if (contact.nodeA.geometry?.firstMaterial?.diffuse.contents as? UIColor) == UIColor.green {
            contact.nodeB.removeFromParentNode()
        } else {
            contact.nodeA.removeFromParentNode()
        }
    }
}

