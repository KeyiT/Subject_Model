package artisynth.models.subjectFrank;

import java.awt.Color;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.inverse.TargetPoint;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointList;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.modelOrderReduction.ReadWrite;
import artisynth.models.subjectFrank.subjects.SwallowPatient;
import maspack.geometry.PolygonalMesh;
import maspack.interpolation.Interpolation.Order;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;

public class TemplateModelTest extends TemplateModel{

   protected RenderableComponentList<PointList> myMarkerList = 
   new RenderableComponentList<PointList> (PointList.class, "TrackingMarkers"); 
   protected int myMarkerSize = 0;
   
// ------------------------- properties ------------------------------//
   public static PropertyList myProps = new PropertyList (TemplateModelTest.class, TemplateModel.class);

   @Override
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   static {
      myProps.add ("targetPositions", "target positions", new VectorNd ());
   }

   public void setTargetPositions (VectorNd tgtPos) {
     int idx = 0;
     for (PointList list : myMarkerList) {
         for (Object mc : list) {
            Point pnt = (Point)mc;
            Point3d pos = new Point3d ();
            tgtPos.getSubVector (idx*3, pos);
            pnt.setTargetPosition (pos);
            idx++;
         }
      }
   }

   public VectorNd getTargetPositions () {

      VectorNd tgtPos = new VectorNd (myMarkerSize*3);
      int idx = 0;
      for (PointList list : myMarkerList) {
          for (Object mc : list) {
             Point pnt = (Point)mc;
             Point3d pos = new Point3d (pnt.getTargetPosition ());
             tgtPos.setSubVector (idx*3, pos);
             idx++;
          }
       }
      return tgtPos;
   }
   


   public void build (String [] args) {
      super.build (args);
      
      attachFaceToThyroid (getFaceThyroidNodes(true));
      attachPharynxToThyroid ();
      addAirway ();
      
      defineCollision ();
      
      forwardTest (0);
      //generateMarkerPathByForward (2, false);
      //inverseTest ();
   }
   
   public void attach (DriverInterface driver) {
      super.attach (driver);
      ArtisynthPath.setWorkingDir ( new File (
         this.getSubjectPath () + "/data/test/result"));
      
      getMainViewer ().setEye (new Point3d (0.0931725, -0.634499, 0.147677));
      getMainViewer ().setCenter (new Point3d (0.09, 0, 0.1));
      RenderProps.setVisible (pharynx.getMuscleBundles (), false);
      RenderProps.setVisible (softPalate.getMuscleBundles (), false);
      RenderProps.setLineWidth (fems, 0);
      for (MuscleBundle mus : externalMuscles) {
         RenderProps.setLineColor (mus, new Color (255, 30, 30));
      }
      for  (MuscleBundle mus : ligaments) {
         RenderProps.setLineColor (mus, new Color (255, 30, 30));
      }
   }
   
   protected void forwardTest (int probeIndex) {
      ArrayList<NumericInputProbe> inputs = 
      forwardTestMuscleInputProbe (getSubjectPath() + "/data/test");
      addInputProbe (inputs.get (probeIndex));
   }
   
   protected void generateMarkerPathByForward (int probeIndex, boolean rewriteMarkers) {
      mechModel.add (myMarkerList);
      RenderProps.setPointStyle (myMarkerList, PointStyle.SPHERE);
      RenderProps.setPointRadius (myMarkerList, 0.001);
      RenderProps.setPointColor (myMarkerList, Color.GREEN);
      
      ArrayList<NumericInputProbe> inputs = 
      forwardTestMuscleInputProbe (getSubjectPath() + "/data/test");
      addInputProbe (inputs.get (probeIndex));
      
      MatrixNd tongueMat = new MatrixNd ();
      MatrixNd jawMat = new MatrixNd ();
      MatrixNd faceMat = new MatrixNd ();
      if (rewriteMarkers) {
         tongueMat = super.writeFemSampledMarkers ("tongue",
            getSubjectPath() + "/data/test/tongueSamplingMesh", 
            null /*getSubjectPath() + "/data/test/tongueTrackMarkers"*/, Integer.MAX_VALUE, -1);
         
         jawMat = super.writeBoneSampledMarkers ("jaw", 
             ArtisynthPath.getSrcRelativePath (SwallowPatient.class, "swallow_CT04mm/registration/jaw/jawSourceMesh"), 
             null /*getSubjectPath() + "/data/test/jawTrackMarkers"*/, (int)(tongueMat.rowSize () / 3.0), -1);
         
         PolygonalMesh faceMesh = 
         SubjectModel.readMeshWithoutSuffix (this.getSubjectPath () + "/data/test/faceSamplingMesh");
         PolygonalMesh tongueMesh = 
         SubjectModel.readMeshWithoutSuffix (this.getSubjectPath () + "/data/test/tongueSamplingMesh");
         faceMat = super.writeFemSampledMarkers ("face", 
            getSubjectPath() + "/data/test/faceSamplingMesh", 
            getSubjectPath() + "/data/test/faceTrackMarkers",
            (int)Math.round (tongueMat.rowSize () * faceMesh.computeArea () / tongueMesh.computeArea ()),
            -1);
         
      }
      else {
         try {
            ReadWrite.readMatrix (tongueMat, getSubjectPath()+"/data/test/tongueTrackMarkers");
            ReadWrite.readMatrix (jawMat, getSubjectPath()+"/data/test/jawTrackMarkers");
            ReadWrite.readMatrix (faceMat, getSubjectPath()+"/data/test/faceTrackMarkers");
         }
         catch (IOException e) {
            e.printStackTrace();
            return;
         }
      }
      
      PointList<FemMarker> faceList = new PointList<FemMarker> (
      FemMarker.class, "faceTrackMarkers");
      for (int i = 0; i < faceMat.rowSize (); i++) {
         Point3d pos = new Point3d ();
         Point3d pos1 = new Point3d ();
         faceMat.getRow (i, pos);
         
         FemMarker marker = new FemMarker (pos);
         marker.setFromFem (face);
         faceList.add (marker);
         myMarkerSize ++;
      }
      myMarkerList.add (faceList);
      
      PointList<FemMarker> pntList = new PointList<FemMarker> (
      FemMarker.class, "tongueTrackMarkers");
      for (int i = 0; i < tongueMat.rowSize (); i++) {
         Point3d pos = new Point3d ();
         Point3d pos1 = new Point3d ();
         tongueMat.getRow (i, pos);
         
         FemMarker marker = new FemMarker (pos);
         marker.setFromFem (tongue);
         pntList.add (marker);
         myMarkerSize ++;
      }
      myMarkerList.add (pntList);
      
      PointList<FrameMarker> markerList = new PointList<FrameMarker> (
      FrameMarker.class, "jawTrackMarkers");
      for (int i = 0; i < jawMat.rowSize (); i++) {
         Point3d pos = new Point3d ();
         jawMat.getRow (i, pos);
         pos.inverseTransform (rbs.get ("jaw").getPose ());
         FrameMarker mkr = new FrameMarker (pos);
         mkr.setFrame (rbs.get ("jaw"));
         markerList.add (mkr);
         myMarkerSize ++;
      }
      myMarkerList.add (markerList);
      
      NumericOutputProbe output = new NumericOutputProbe (
         this, "targetPositions", inputs.get (probeIndex).getStartTime (), 
         inputs.get (probeIndex).getStopTime (), getMaxStepSize ());
      addOutputProbe (output);
   }
   
   ComponentList<MuscleExciter> trackExciters = new ComponentList <MuscleExciter> (MuscleExciter.class, "targetExciter");
   TrackingController trackingController = new TrackingController(mechModel, "trackingController");
   
   protected void inverseTest() 
   {
      mechModel.add (myMarkerList);
      RenderProps.setPointStyle (myMarkerList, PointStyle.SPHERE);
      RenderProps.setPointRadius (myMarkerList, 0.001);
      RenderProps.setPointColor (myMarkerList, Color.GREEN);
      
      MatrixNd tongueMat = new MatrixNd ();
      MatrixNd jawMat = new MatrixNd ();
      MatrixNd faceMat = new MatrixNd ();
      try {
         ReadWrite.readMatrix (tongueMat, getSubjectPath()+"/data/test/tongueTrackMarkers");
         ReadWrite.readMatrix (jawMat, getSubjectPath()+"/data/test/jawTrackMarkers");
         ReadWrite.readMatrix (faceMat, getSubjectPath()+"/data/test/faceTrackMarkers");
      }
      catch (IOException e) {
         e.printStackTrace();
         return;
      }
      
      PointList<FemMarker> faceList = new PointList<FemMarker> (
      FemMarker.class, "faceTrackMarkers");
      for (int i = 0; i < faceMat.rowSize (); i++) {
         Point3d pos = new Point3d ();
         Point3d pos1 = new Point3d ();
         faceMat.getRow (i, pos);
         
         FemMarker marker = new FemMarker (pos);
         marker.setFromFem (face);
         faceList.add (marker);
         marker.setName ("TM" + myMarkerSize);
         trackingController.addMotionTarget (marker);
         myMarkerSize ++;
      }
      myMarkerList.add (faceList);
      
      PointList<FemMarker> pntList = new PointList<FemMarker> (
      FemMarker.class, "tongueTrackMarkers");
      for (int i = 0; i < tongueMat.rowSize (); i++) {
         Point3d pos = new Point3d ();
         Point3d pos1 = new Point3d ();
         tongueMat.getRow (i, pos);
         
         FemMarker marker = new FemMarker (pos);
         marker.setFromFem (tongue);
         pntList.add (marker);
         marker.setName ("TM" + myMarkerSize);
         trackingController.addMotionTarget (marker);
         myMarkerSize ++;
      }
      myMarkerList.add (pntList);
      
      PointList<FrameMarker> markerList = new PointList<FrameMarker> (
      FrameMarker.class, "jawTrackMarkers");
      for (int i = 0; i < jawMat.rowSize (); i++) {
         Point3d pos = new Point3d ();
         jawMat.getRow (i, pos);
         pos.inverseTransform (rbs.get ("jaw").getPose ());
         FrameMarker mkr = new FrameMarker (pos);
         mkr.setFrame (rbs.get ("jaw"));
         markerList.add (mkr);
         mkr.setName ("TM" + myMarkerSize);
         trackingController.addMotionTarget (mkr);
         myMarkerSize ++;
      }
      myMarkerList.add (markerList);
      
      
      MuscleExciter openers = new MuscleExciter ("JawOpeners");
      MuscleExciter closers = new MuscleExciter ("JawClosers");
      trackExciters.add (openers);
      trackExciters.add (closers);
      mechModel.add (trackExciters);
      for (Object obj : exciters.get ("FaceExciters")) {
         MuscleExciter ex = (MuscleExciter)obj;
         trackExciters.add (ex);
      }
      
      for (Object obj : exciters.get ("TongueExciters")) {
         MuscleExciter ex = (MuscleExciter)obj;
         trackExciters.add (ex);
      }
      
      for (Object obj : exciters.get ("ExternalExciters")) {
         MuscleExciter ex = (MuscleExciter)obj;
         
         if (ex.getName ().equalsIgnoreCase ("AD") ||
         ex.getName ().equalsIgnoreCase ("PD") ||
         ex.getName ().equalsIgnoreCase ("SH")) {
            openers.addTarget (ex);
         }
         
         if (ex.getName ().equalsIgnoreCase ("AT") ||
         ex.getName ().equalsIgnoreCase ("PT") ||
         ex.getName ().equalsIgnoreCase ("MT") ||
         ex.getName ().equalsIgnoreCase ("DM") ||
         ex.getName ().equalsIgnoreCase ("SM") ||
         ex.getName ().equalsIgnoreCase ("MP")) {
            closers.addTarget (ex);
         }
      }
      
      for (MuscleExciter ex : trackExciters) {
         trackingController.addExciter(ex);
      }
  
      trackingController.setProbeDuration (0.4);
      trackingController.addDampingTerm (0.0001);
      trackingController.addL2RegularizationTerm(/*l2norm*/0.001);
      trackingController.setMaxExcitationJump (0.1);
      trackingController.getMotionTargetTerm ().setNormalizeH (true);
      trackingController.createProbesAndPanel (this);
      addController(trackingController);
      
      trackingController.setTargetsPointRadius (0.001);
      trackingController.setTargetsVisible (true);
      
      NumericInputProbe input = (NumericInputProbe) getInputProbes ().get (0);
      input.setAttachedFileName (this.getSubjectPath () + "/data/test/produceU.inverseProbe");
      

      try {
         input.load ();
         input.setExtendData (true);
         input.setInterpolationOrder (Order.Linear);
      }
      catch (IOException e) {
         e.printStackTrace();
      }
      input.setActive (true);
   }
   
   public void report () {
      PointList<TargetPoint> tgtPnts = trackingController.getTargetPoints ();
      MatrixNd rMat0=  new MatrixNd (0, 3);
      MatrixNd rMat1 = new MatrixNd (0, 3);
      
      int idx = 0;
      for (TargetPoint tgtPnt : tgtPnts) {
         rMat1.setSize (idx + 1, 3);
         rMat1.setRow (idx++, tgtPnt.getPosition ());
      }
      
      idx = 0;
      for (PointList list : myMarkerList) {
         for (Object obj : list) {
            Point src = (Point)obj;
            rMat0.setSize (idx + 1, 3);
            rMat0.setRow (idx++, src.getPosition ());
         }
      }
      
      double max = 0;
      double total = 0;
      for (int i = 0; i < rMat0.rowSize (); i++) {
         Point3d pnt = new Point3d ();
         Point3d tgt = new Point3d ();
         rMat0.getRow (i, pnt);
         rMat1.getRow (i, tgt);
         
         double dis = tgt.distance (pnt);
         if (dis > max) {
            max =  dis;
         }
         total += dis;
      }
      
      double mean = total/(double)rMat0.rowSize ();
      
      double dev = 0;
      for (int i = 0; i < rMat0.rowSize (); i++) {
         Point3d pnt = new Point3d ();
         Point3d tgt = new Point3d ();
         rMat0.getRow (i, pnt);
         rMat1.getRow (i, tgt);
         
         double dis = tgt.distance (pnt);
         double err = dis - mean;
         
         dev += err * err;
      }
      dev = dev / (double)rMat0.rowSize ();
      dev = Math.sqrt (dev);
      
      System.out.println ("mean: " + mean);
      System.out.println ("max: " + max);
      System.out.println ("dev: " + dev);
      System.out.println (rMat0);
   }
   
   public static void computeTrackMeanError () {
      MatrixNd rMat0=  new MatrixNd ();
      MatrixNd rMat1 = new MatrixNd ();
      try {
         ReadWrite.readMatrix (rMat0, SwallowPatient.class, "/speech_MRI/data/test/result/produceA_inverseEndPositions");
         ReadWrite.readMatrix (rMat1, SwallowPatient.class, "/speech_MRI/data/test/result/produceA_targetEndPositions");
      }
      catch (IOException e) {
         e.printStackTrace();
      }
      
      VectorNd tgts = new VectorNd (rMat1);
      double max = 0;
      double total = 0;
      for (int i = 0; i < rMat0.rowSize (); i++) {
         Point3d pnt = new Point3d ();
         Point3d tgt = new Point3d ();
         rMat0.getRow (i, pnt);
         tgts.getSubVector (i*3, tgt);
         
         double dis = tgt.distance (pnt);
         if (dis > max) {
            max =  dis;
         }
         total += dis;
      }
      
      double mean = total/(double)rMat0.rowSize ();
      
      double dev = 0;
      for (int i = 0; i < rMat0.rowSize (); i++) {
         Point3d pnt = new Point3d ();
         Point3d tgt = new Point3d ();
         rMat0.getRow (i, pnt);
         tgts.getSubVector (i*3, tgt);
         
         double dis = tgt.distance (pnt);
         double err = dis - mean;
         
         dev += err * err;
      }
      dev = dev / (double)rMat0.rowSize ();
      dev = Math.sqrt (dev);
      
      System.out.println ("mean: " + mean);
      System.out.println ("max: " + max);
      System.out.println ("dev: " + dev);
   }
   
   public static void main (String [] args) {
      computeTrackMeanError ();
   }
   
}
