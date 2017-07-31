package artisynth.models.subjectFrank;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import javax.swing.JButton;
import javax.swing.JMenuItem;
import javax.swing.JSeparator;

import org.python.google.common.io.Files;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.femmodels.SkinMeshBody;
import artisynth.core.femmodels.VtkInputOutput;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.PeckAxialMuscle;
import artisynth.core.mechmodels.DynamicAttachment;
import artisynth.core.mechmodels.MeshComponentList;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointFrameAttachment;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.RenderableComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.fluid1d.fileIO.VTK_IO;
import artisynth.models.frank2.FrankModel2;
import artisynth.models.modelOrderReduction.ReadWrite;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDeformer;
import artisynth.models.swallowingRegistrationTool.ICP.ICPManager;
import artisynth.models.swallowingRegistrationTool.infoUtilities.UniformMeshFeatureSubsampler;
import artisynth.models.swallowingRegistrationTool.transformers.AffineModelTransformer;
import artisynth.models.swallowingRegistrationTool.utilities.FemModelAgent;
import artisynth.models.swallowingRegistrationTool.utilities.MeshModelAgent;
import maspack.geometry.Face;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.geometry.io.GenericMeshReader;
import maspack.geometry.io.VtkAsciiReader;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.ImproperStateException;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.render.ColorMapProps;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.Shading;

/**
 * 
 * @author KeyiTang
 *
 */
public abstract class SubjectModel extends FrankModel2 implements ActionListener{

   // path
   private final static String SUBJECT_DATA_ROOT_PATH =
   ArtisynthPath.getSrcRelativePath (SubjectModel.class, "subjects/");
   String subjectDataFolder;

   private LinkedList<ModelComponent> myRemoveList = new LinkedList<ModelComponent>();

   // model 
   protected final HashSet<String> frankComponentNames = new HashSet<String> ();

   // frank rigid body
   protected RenderableComponentList<RigidBody> rbs;
   protected RenderableComponentList<FemMuscleModel> fems;
   protected RenderableComponentList<MuscleBundle> externalMuscles;
   protected RenderableComponentList<MuscleBundle> ligaments;
   protected MeshComponentList<MeshModelAgent> externalMeshAgents = 
   new MeshComponentList<MeshModelAgent> (MeshModelAgent.class, "External Mesh", "EM");
   protected ComponentList<ComponentList> exciters;

   // agent
   private ArrayList <MeshModelAgent> rbAgents = new ArrayList <MeshModelAgent> ();
   private ArrayList <FemModelAgent> femAgents = new ArrayList <FemModelAgent> ();

   // skin mesh
   protected SkinMeshBody airway;
   protected MeshModelAgent airwayAgent;

   public enum FrankRigidBodyComponents {
      jaw, maxilla, hyoid, thyroid, cricoid, epiglottis, cranium,
      cuneiform_L, cuneiform_R, arytenoid_L, arytenoid_R, NULL
   }
   public enum FrankFemComponents {
      tongue, softPalate, pharynx, larynx, face, NULL
   }

   public enum SelectedICPManager {
      AFFINE, FFD, ALL
   }

   private FrankRigidBodyComponents frankRbs = 
   FrankRigidBodyComponents.NULL;
   private FrankFemComponents frankFems = 
   FrankFemComponents.NULL;
   private SelectedICPManager selICP = SelectedICPManager.ALL;

   // ------------------------- properties ------------------------------//
   public static PropertyList myProps = new PropertyList (SubjectModel.class, FrankModel2.class);

   @Override
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   static {
      myProps.add ("selectRbComp", "select Frank rigid-Body", FrankRigidBodyComponents.NULL);
      myProps.add ("selectFemComp", "select Frank deformable-body", FrankFemComponents.NULL);
      myProps.add ("selectICP", "select ICP manager", SelectedICPManager.ALL);
      myProps.add ("exciterActivations", "exciter activation values", new VectorNd ());
   }

   public void setSelectRbComp (FrankRigidBodyComponents rbComp) {
      frankRbs = rbComp;
   }

   public FrankRigidBodyComponents getSelectRbComp () {
      return frankRbs;
   }

   public void setSelectFemComp (FrankFemComponents femComp) {
      frankFems = femComp;
   }

   public FrankFemComponents getSelectFemComp () {
      return frankFems;
   }

   public void setSelectICP (SelectedICPManager icp) {
      selICP = icp;
   }

   public SelectedICPManager getSelectICP () {
      return selICP;
   }

   public void setExciterActivations (VectorNd as) {
      int exIdx = 0;
      for (ComponentList list : exciters) {
         ComponentList<MuscleExciter> exs = (ComponentList<MuscleExciter>)list;
         for (MuscleExciter ex : exs) {
            ex.setExcitation (as.get (exIdx++));
         }
      }
   }

   public VectorNd getExciterActivations () {
      VectorNd as = new VectorNd ();
      int exIdx = 0;
      for (ComponentList list : exciters) {
         ComponentList<MuscleExciter> exs = (ComponentList<MuscleExciter>)list;
         for (MuscleExciter ex : exs) {
            as.setSize (exIdx + 1);
            as.set (exIdx++, ex.getExcitation ());
         }
      }
      return as;
   }

   public void build (String [] args) {
      try {
         super.build (args);

         // get frank rigid bodies
         rbs = (RenderableComponentList<RigidBody>) 
         mechModel.get ("RigidBodies");
         fems = (RenderableComponentList<FemMuscleModel>)
         mechModel.get ("DeformableBodies");
         externalMuscles = (RenderableComponentList<MuscleBundle>)
         mechModel.get ("ExternalMuscles");
         ligaments = (RenderableComponentList<MuscleBundle>)
         mechModel.get ("Ligaments");
         exciters = (ComponentList<ComponentList>)
         mechModel.get ("MuscleExciters");
         // get name list of rigid bodies
         // and FEM soft tissues
         updateFrankComponentNameList ();
         setName (getSubjectName ());
         // 
         addRbSlaveButton.addActionListener (this);
         addFemSlaveButton.addActionListener (this);
         renderMasterAttachmentButton.addActionListener (this);
         updateMasterAttachmentButton.addActionListener (this);
         saveMasterButton.addActionListener (this);
         saveSlaveButton.addActionListener (this);
         transformSlaveInBatchButton.addActionListener (this);

         if (makeExternalSourceMeshAgents ()) {
            addRenderable(externalMeshAgents);
            externalMeshAgents.getRenderProps ().setVisible (false);
         }

         mechModel.clearCollisionBehaviors ();
         myRemoveList.clear ();
         removeComponents (myRemoveList);
         if (myRemoveList.size () != 0) {
            int [] myRemoveIndices = new int[myRemoveList.size()];
            ComponentUtils.removeComponents (myRemoveList, myRemoveIndices);
         }
      }
      catch (IOException e) {
         e.printStackTrace();
         System.err.println (
         "Failed to load Frank model properly!");
      }

   }

   /**
    * add the components into <tt>removeList<tt>.
    * @param removeList
    */
   protected void removeComponents (LinkedList<ModelComponent> removeList) {
      // just hook here
   }

   /**
    * save the name list of all frank rigid bodies
    * and FEM components;
    */
   private void updateFrankComponentNameList () {
      frankComponentNames.clear ();
      for (RigidBody rb: rbs) {
         frankComponentNames.add (rb.getName ());
      }
      for (FemMuscleModel fem: fems) {
         frankComponentNames.add (fem.getName ());
      }
   }

   private void updateFrankComponentList () {
      updateFrankComponentNameList ();
   }

   /**
    * Retrieve path for this subject; Before calling this method, 
    * the name of this model should be set as the subject name;
    * @return 
    * subjects data directory / model name (subject name)
    */
   public String getSubjectPath () {
      String path = new String (SUBJECT_DATA_ROOT_PATH 
         + getSubjectName ());
      return path;
   }

   /**
    * Retrieve path for this subject; Before calling this method, 
    * the name of this model should be set as the subject name;
    * @param rb Frank <code>RigidBody<code> component
    * @return 
    * subjects data directory / model name (subject name) / 
    * <tt>rb</tt> name
    */
   public String getSubjectRigidBodyPath (RigidBody rb) {
      if (!rbs.contains (rb)) {
         throw new NullPointerException (
         "Not a Frank rigid body component");
      }
      String path = new String (
         getSubjectPath () + "/" + rb.getName ());
      return path;
   }

   /**
    * Retrieve path for this subject; Before calling this method, 
    * the name of this model should be set as the subject name;
    * @param fem Frank <code>FemModel3d<code> component
    * @return 
    * subjects data directory / model name (subject name)
    * <tt>fem</tt> name
    */
   public String getSubjectDeformableBodyPath (FemModel3d fem) {
      if (!fems.contains (fem)) {
         throw new NullPointerException (
         "Not a Frank deformable body component");
      }
      String path = new String (
         getSubjectPath () + "/" + fem.getName ());
      return path;
   }

   /**
    * This is to import external mesh as source mesh of registration;
    * If no external mesh needed for registration, return null;
    * @return
    * A <code>MeshModelAgent</code> whose mesh will be used as source
    * mesh of registration. The name of returned agent must be the same 
    * as it's corresponding target mesh folder name;
    */
   private boolean makeExternalSourceMeshAgents () {
      ArrayList<String> names = new ArrayList<String> ();
      getExternalDataNames(names);

      externalMeshAgents.clear ();
      for (String name : names) {
         String subPath = getSubjectPath ();
         String dataPath= subPath + "/registration/" + name;
         File dataDir = new File (dataPath);
         if (! (dataDir.exists () && dataDir.isDirectory ())) {
            System.err.println ("External data directory for " + 
            name + " not found!");
            continue;
         }
         String srcPath = dataPath + "/" + name + "SourceMesh";
         PolygonalMesh mesh = readMeshWithoutSuffix (srcPath);
         if (mesh != null) {
            MeshModelAgent agent = new MeshModelAgent ();
            agent.represent (mesh);
            agent.setName (name);
            externalMeshAgents.add (agent);
         }
      }

      if (externalMeshAgents.size () != 0)
         return true;
      return false;
   }

   /**
    * load subject data from files; 
    */
   public void loadSubjectData () {

      String sbjPath = getSubjectPath ();

      File sbjDir = new File(sbjPath);
      if (! (sbjDir.exists () && sbjDir.isDirectory ()) ) {
         throw new DataLoadException (
            "No data for subject " + getSubjectName());
      }

      // frank component --> agent
      HashMap <RigidBody, MeshModelAgent> oldRbs = 
      new HashMap <RigidBody, MeshModelAgent> ();
      HashMap <FemMuscleModel, FemModelAgent> oldFems = 
      new HashMap <FemMuscleModel, FemModelAgent> ();

      for (MeshModelAgent agent : rbAgents) {
         oldRbs.put ((RigidBody)agent.getClient (), agent);
      }
      for (FemModelAgent agent : femAgents) {
         oldFems.put ((FemMuscleModel)agent.getClient (), agent);
      }
      rbAgents.clear ();
      femAgents.clear ();

      // load bone
      for (RigidBody rb : rbs) {
         String cpPath = getSubjectRigidBodyPath (rb);
         File cpDir = new File (cpPath);
         if (! (cpDir.exists () && cpDir.isDirectory ()) ) {
            // remove from subject render list
            if (oldRbs.containsKey (rb)) {
               removeRenderable (oldRbs.get (rb));
               oldRbs.remove (rb);
            }
         }
         else {
            if (oldRbs.containsKey (rb)) {
               MeshModelAgent agent = oldRbs.get (rb);
               agent.setClientMechModel (mechModel);
               loadRigidBodyData (agent, getName ());
               rbAgents.add (agent);
            }
            else {
               MeshModelAgent agent = new MeshModelAgent (rb);
               agent.setClientMechModel (mechModel);
               agent.setName (rb.getName ()+"Subject");
               loadRigidBodyData (agent, getName ());
               addRenderable(agent);
               rbAgents.add (agent);
            }
         }
      }


      for (FemMuscleModel fem : fems) {
         String cpPath = getSubjectDeformableBodyPath (fem);
         File cpDir = new File (cpPath);
         if (! (cpDir.exists () && cpDir.isDirectory ()) ) {
            // remove from subject render list
            if (oldFems.containsKey (fem)) {
               removeRenderable (oldFems.get (fem));
               oldFems.remove (fem);
            }
         }
         else {
            if (oldFems.containsKey (fem)) {
               FemModelAgent agent = oldFems.get (fem);
               agent.setClientMechModel (mechModel);
               loadFemData (agent, getName ());
               femAgents.add (agent);
            }
            else {
               FemModelAgent agent = new FemModelAgent (fem);
               agent.setClientMechModel (mechModel);
               agent.setName (fem.getName ()+"Subject");
               loadFemData (agent, getName ());
               addRenderable(agent);
               femAgents.add (agent);
            }
         }
      }

      for (FemModelAgent agent : femAgents) {
         agent.updateAttachmentPos ();
      }
   }

   // TODO: generate button 
   public void clearSubjectData () {
      // clear bone
      for (MeshModelAgent rb : rbAgents) {
         removeRenderable (rb);
      }
      rbAgents.clear ();

      for (FemModelAgent fem : femAgents) {
         removeRenderable (fem);
      }
      femAgents.clear ();
   }

   /**
    * Load subject mesh data and pass it to the <tt>boneAgent</tt>;
    * Load subject muscle attachment data pass it to the <tt>boneAgent</tt>;
    * 
    * @see {@link MeshModelAgent#copyMesh}
    * @see {@link MeshModelAgent#setIsolatedPointFrameAttachmentSlaveAgents}
    * 
    * @param boneAgent a <code>RigidBody<code> agent <code>represent</code>s 
    * one of the Frank <code>RigidBody</code> components
    * 
    * @param subjectName the subject folder name
    */
   public void loadRigidBodyData (MeshModelAgent boneAgent, String subjectName) {
      if (! rbs.contains (boneAgent.getClient ())) {
         throw new NullPointerException (
         "Client is not Frank rigid body component!");
      }

      String boneName = boneAgent.getClient ().getName ();
      String dataPath = getSubjectRigidBodyPath (
         (RigidBody)boneAgent.getClient ());

      // mesh
      PolygonalMesh meshData = readMeshWithoutSuffix(
         dataPath + "/" + boneName + "Mesh");
      if (meshData != null) {
         boneAgent.copyMesh (meshData);
      }
      else {
         boneAgent.getRenderProps ().setAlpha (0.5);
      }

      // muscle attachment
      File musAtmFile = new File (dataPath+"/muscleAttachment.txt");
      if (musAtmFile.exists () && musAtmFile.isFile ()) {
         MatrixNd data = new MatrixNd  ();
         try {
            ReadWrite.readMatrix (data, dataPath+"/muscleAttachment.txt");
         }
         catch (IOException e) {
            e.printStackTrace ();
            throw new DataLoadException (
            "Failed to read external muscle attachment data!");
         }
         boneAgent.setIsolatedPointFrameAttachmentSlaveAgents (data);
      }
   }


   public void loadFemData (FemModelAgent femAgent, String Folder) {
      if (! fems.contains (femAgent.getClient ())) {
         throw new NullPointerException (
         "Client is not Frank deformable body component!");
      }

      String femName = femAgent.getClient ().getName ();
      String dataPath = getSubjectDeformableBodyPath (
         (FemModel3d)femAgent.getClient ());

      // mesh
      FemModel3d femData = new FemModel3d ();

      readFemWithoutSuffix(femData, dataPath + "/" + femName + "FEM");
      System.out.println (femAgent.getName ());
      //System.out.println (femData.getElements ().size () + " ------ " + femAgent.getElements ().size ()+ " ------ " + femAgent.getClient ().getElements ().size ());
      //System.out.println (femData.getNodes ().size () + " ------ " + femAgent.getNodes ().size ());

      if (femData.getElements ().size () == femAgent.getElements ().size () &&
      femData.getNodes ().size () == femAgent.getNodes ().size ()) {
         femAgent.copyVolumeMesh (femData);
      }
      else {
         femAgent.getRenderProps ().setAlpha (0);
      }
      // TODO:attachments
   }

   // TODO: 
   /**
    * 
    */
   public void makeSubjectModel () {

      // move fixed muscle points
      fixedMusclePointGrow ();

      System.out.println ("\n rb agent commit ...");
      for (MeshModelAgent agent : rbAgents) {
         System.out.println (agent.getClient ().getName () + "...");
         agent.commit ();
      }

      if (airway != null) {
         airwayAgent = new MeshModelAgent (airway);   
         //airway.clearAttachments ();
         airwayAgent.updateSkinMesh (femAgents);
         //airway.computeWeights ();
      }

      System.out.println ("\n fem agent commit ...");
      for (FemModelAgent agent : femAgents) {
         System.out.println (agent.getClient ().getName () + "...");
         agent.commit ();
      }

      System.out.println ("\nrb model grow");
      for (MeshModelAgent agent : rbAgents) {
         System.out.println (agent.getClient ().getName () + "...");
         agent.grow ();
      }
      System.out.println ("\nfem model grow");
      for (FemModelAgent agent : femAgents) {
         System.out.println (agent.getClient ().getName () + "...");
         agent.grow ();
      }

      System.out.println ("reset muscle reset length ...");
      // reset point-frame attachment
      for (RigidBody rb : rbs) {
         if (rb.getMasterAttachments () == null) {
            continue;
         }
         for (DynamicAttachment att : rb.getMasterAttachments ()) {
            if (att instanceof PointFrameAttachment) {
               PointFrameAttachment pntAtt = (PointFrameAttachment)att;

               Point pnt = pntAtt.getPoint ();
               if (pnt == null) {
                  throw new NullPointerException (
                  "Unkown point attachment slave");
               }

               Point3d loc = new Point3d ();
               pntAtt.getPoint ().getPosition (loc);
               loc.inverseTransform (rb.getPose ());
               pntAtt.setLocation (loc);
            }
         }
      }

      mechModel.updatePosState ();
      mechModel.updateAttachmentPos ();

      for (FemMuscleModel fem : fems) {
         // reset rest position of FEM nodes
         for (FemNode3d node : fem.getNodes ()) {
            node.setRestPosition (node.getPosition ());
         }

         // reset muscle bundle rest length
         if (fem instanceof FemMuscleModel) {
            FemMuscleModel client = (FemMuscleModel)fem;
            for (MuscleBundle mb : client.getMuscleBundles ()) {
               double ll0 = 0, ll1 = 0;
               for (Muscle fiber: mb.getFibres ()) {
                  double l0 = fiber.getRestLength ();
                  ll0 += l0;
                  fiber.setRestLengthFromPoints ();
                  double l1 = fiber.getRestLength ();
                  ll1 += l1;
                  if (fiber.getMaterial () instanceof PeckAxialMuscle) {
                     PeckAxialMuscle mat = (PeckAxialMuscle)fiber.getMaterial ();
                     // changing the two parameters below may cause stability issues
                     mat.setOptLength (mat.getOptLength () * l1 / l0);
                     mat.setMaxLength (mat.getMaxLength () * l1 / l0);
                  }
               }
               
               
               // new method: keep the opt and max length consistent within the muscle bundle
               for (Muscle fiber: mb.getFibres ()) {
                  if (fiber.getMaterial () instanceof PeckAxialMuscle) {
                     PeckAxialMuscle mat = (PeckAxialMuscle)fiber.getMaterial ();
                     //mat.setOptLength (mat.getOptLength () * ll1 / ll0);
                     //mat.setMaxLength (mat.getMaxLength () * ll1 / ll0);
                  }
               }
            }
         }
      }

      // reset external muscles rest length
      for (MuscleBundle mus : externalMuscles) {
         double ll0 = 0, ll1 = 0;
         
         for (Muscle fiber : mus.getFibres ()) {
            double l0 = fiber.getRestLength ();
            ll0 += l0;
            fiber.setRestLengthFromPoints ();
            double l1 = fiber.getRestLength ();
            ll1 += l1;
            if (fiber.getMaterial () instanceof PeckAxialMuscle) {
               PeckAxialMuscle mat = (PeckAxialMuscle)fiber.getMaterial ();
               // changing the two parameters below may cause stability issues
              mat.setOptLength (mat.getOptLength () * l1 / l0);
              mat.setMaxLength (mat.getMaxLength () * l1 / l0);
            }
         }
         
         // new method: keep the opt and max length consistent within the muscle bundle
         for (Muscle fiber: mus.getFibres ()) {
            if (fiber.getMaterial () instanceof PeckAxialMuscle) {
               PeckAxialMuscle mat = (PeckAxialMuscle)fiber.getMaterial ();
               //mat.setOptLength (mat.getOptLength () * ll1 / ll0);
               //mat.setMaxLength (mat.getMaxLength () * ll1 / ll0);
            }
         }
      }

      // reset ligaments rest length
      for (MuscleBundle lig : ligaments) {
         for (Muscle fiber : lig.getFibres ()) {
            fiber.setRestLengthFromPoints ();
         }
      }

      mechModel.updatePosState ();
      mechModel.updateAttachmentPos ();
      if (airway != null) {
         airway.updateSlavePos ();
      }

      // render property
      for (RigidBody rb : rbs) {
         rb.getMesh ().notifyVertexPositionsModified ();
         rb.getMesh ().updateFaceNormals ();
      }

      for (FemModelAgent agent : femAgents) {
         RenderProps.setVisible (agent, false);
      }
      for (MeshModelAgent agent : rbAgents) {
         RenderProps.setVisible (agent, false);
      }
      // For test
      //GenericModel.renderAttachedNodes  (pharynx, 0.001, Color.darkGray);
      //GenericModel.renderAttachedNodes(larynx, 0.001, Color.cyan);
      //GenericModel.renderAttachedNodes(tongue, 0, Color.green);
      //GenericModel.renderAttachedNodes(softPalate, 0.001, Color.green);


      if (airway != null) {
         // airway.computeWeights ();
      }
   }

   protected void fixedMusclePointGrow () {

      ArrayList<Point3d> srcPnts = new ArrayList<Point3d> ();
      ArrayList<Point3d> tgtPnts = new ArrayList<Point3d> ();

      AffineTransform3d affine = new AffineTransform3d ();

      Set<FemModel3d> doneFems = new HashSet<FemModel3d> ();

      for (FemModelAgent agent : femAgents) {
         HashMap <FemNode3d, FemNode3d> nodeMap = 
         new HashMap <FemNode3d, FemNode3d> ();
         agent.getNodeMap (nodeMap);

         Set ens = nodeMap.entrySet ();
         Iterator it = ens.iterator ();
         while (it.hasNext ()) {
            Entry<FemNode3d, FemNode3d> me = 
            (Entry<FemNode3d, FemNode3d>)it.next ();

            srcPnts.add (me.getKey ().getPosition ());
            tgtPnts.add (me.getValue ().getPosition ());
         }

         Set<Point> fixedPoints = new HashSet<Point> ();

         if (agent.getClient () instanceof FemMuscleModel) {
            FemMuscleModel client = (FemMuscleModel)agent.getClient ();
            for (MuscleBundle mb : client.getMuscleBundles ()) {
               for (Muscle fiber: mb.getFibres ()) {
                  Point p;
                  p = fiber.getFirstPoint ();
                  if (p instanceof FemNode3d) {
                     FemNode3d no = (FemNode3d)p;
                     if (no.numAdjacentElements () != 0) {
                        continue;
                     }
                  }
                  if (p instanceof FemMarker) {
                     continue;
                  }
                  if (!p.isDynamic ()) {
                     fixedPoints.add (p);
                  }
                  p = fiber.getSecondPoint ();
                  if (p instanceof FemNode3d) {
                     FemNode3d no = (FemNode3d)p;
                     if (no.numAdjacentElements () != 0) {
                        continue;
                     }
                  }
                  if (p instanceof FemMarker) {
                     continue;
                  }
                  if (!p.isDynamic ()) {
                     fixedPoints.add (p);
                  }
               }
            }
         }

         if (fixedPoints.size () != 0) {
            affine.fit (tgtPnts, srcPnts);
            it = fixedPoints.iterator ();
            while (it.hasNext ()) {
               Point me = (Point)it.next ();
               me.getPosition ().transform (affine);
               //me.transformGeometry (affine);
            }
         }

         doneFems.add (agent.getClient ());
      }

      for (MeshModelAgent agent : rbAgents) {
         RigidBody rb = (RigidBody) agent.getClient ();
         MeshBase srcmesh = rb.getMesh ();
         MeshBase tgtmesh = agent.getMesh ();
         if (tgtmesh.numVertices () != srcmesh.numVertices ()) {
            throw new ModelGrowException ("Incompatible agent!");
         }

         for (Vertex3d vtx : srcmesh.getVertices ()) {
            Point3d pnt = new Point3d (vtx.getWorldPoint ());
            srcPnts.add (pnt);
         }
         for (Vertex3d vtx : tgtmesh.getVertices ()) {
            Point3d pnt = new Point3d (vtx.getWorldPoint ());
            tgtPnts.add (pnt);
         }
      }

      Set<Point> fixedPoints = new HashSet<Point> ();

      for (FemMuscleModel fem : fems) {
         if (doneFems.contains ((FemModel3d)fem)) {
            continue;
         }
         FemMuscleModel client = (FemMuscleModel)fem;
         for (MuscleBundle mb : client.getMuscleBundles ()) {
            for (Muscle fiber: mb.getFibres ()) {
               Point p;
               p = fiber.getFirstPoint ();
               if (p instanceof FemNode3d) {
                  FemNode3d no = (FemNode3d)p;
                  if (no.numAdjacentElements () != 0) {
                     continue;
                  }
               }
               if (p instanceof FemMarker) {
                  continue;
               }
               if (!p.isDynamic ()) {
                  fixedPoints.add (p);
               }
               p = fiber.getSecondPoint ();
               if (p instanceof FemNode3d) {
                  FemNode3d no = (FemNode3d)p;
                  if (no.numAdjacentElements () != 0) {
                     continue;
                  }
               }
               if (p instanceof FemMarker) {
                  continue;
               }
               if (!p.isDynamic ()) {
                  fixedPoints.add (p);
               }
            }
         }
      }

      // reset external muscle rest length
      for (MuscleBundle mus : externalMuscles) {
         for (Muscle fiber : mus.getFibres ()) {
            Point p;
            p = fiber.getFirstPoint ();
            if (!p.isDynamic ()) {
               fixedPoints.add (p);
            }
            p = fiber.getSecondPoint ();
            if (!p.isDynamic ()) {
               fixedPoints.add (p);
            }
         }
      }

      for (MuscleBundle mus : ligaments) {
         for (Muscle fiber : mus.getFibres ()) {
            Point p;
            p = fiber.getFirstPoint ();
            if (!p.isDynamic ()) {
               fixedPoints.add (p);
            }
            p = fiber.getSecondPoint ();
            if (!p.isDynamic ()) {
               fixedPoints.add (p);
            }
         }
      }

      // cranium and ground attachments
      List<RigidBody> myrbs = new ArrayList<RigidBody> ();
      myrbs.add (rbs.get ("cranium"));
      myrbs.add (rbs.get ("ground"));

      for (RigidBody rb : myrbs) {
         if (rb.getMasterAttachments () != null) {
            for (DynamicAttachment att : rb.getMasterAttachments ()) {
               if (att instanceof PointFrameAttachment) {
                  PointFrameAttachment patt = (PointFrameAttachment)att;
                  Point pnt = patt.getPoint ();
                  if (pnt != null ) {
                     if (pnt instanceof FemNode3d) {
                        if (((FemNode3d)pnt).numAdjacentElements () != 0) {
                           continue;
                        }
                     }
                     fixedPoints.add (pnt);
                  }
               }
            }
         }
      }


      if (fixedPoints.size () != 0) {
         affine.fit (tgtPnts, srcPnts);
         Iterator it = fixedPoints.iterator ();
         while (it.hasNext ()) {
            Point me = (Point)it.next ();
            me.transformGeometry (affine);
         }
      }


   }

   // -------------------------------------
   // implement registration
   // -------------------------------------
   private HashSet<MeshModelAgent> masterRbAgent = new HashSet<MeshModelAgent>();
   private HashSet<FemModelAgent> masterFemAgent = new HashSet<FemModelAgent> ();
   private HashSet<MeshModelAgent> masterExternalMeshAgents = new HashSet<MeshModelAgent> ();

   private ICPManager icp;
   private ControlPanel myRegPanel = new ControlPanel ();

   // frank componens --> agents
   private HashMap <TransformableGeometry, TransformableGeometry> icpSlaves = 
   new HashMap <TransformableGeometry, TransformableGeometry> ();



   private boolean registrationGenerated = false;

   /**
    * Get the selected components, make registration agents for each of them;
    * <p>
    * @return if master component found return true, otherwise return false;
    */
   protected boolean selectMasterAgent () {
      masterRbAgent.clear ();
      masterFemAgent.clear ();
      masterExternalMeshAgents.clear();
      boolean flag = false;

      for (RigidBody rb : rbs) {
         if (rb.isSelected ()) {
            MeshModelAgent agent = new MeshModelAgent (rb);
            masterRbAgent.add (agent);
            flag = true;
         }
      }
      for (FemMuscleModel fem: fems) {
         if (fem.isSelected ()) {
            FemModelAgent agent = new FemModelAgent (fem);
            masterFemAgent.add (agent);
            flag = true;
         }
      }
      for (MeshModelAgent ext : externalMeshAgents) {
         if (ext.isSelected ()) {
            MeshModelAgent agent = new MeshModelAgent (ext);
            masterExternalMeshAgents.add (agent);
            flag = true;
         }
      }

      return flag;
   }

   public void generateRegistrationManager () {
      setMaxStepSize (0.01);
      setMinStepSize (0.01);

      // remove previous registration manager
      clearRegistrationManager ();
      clearSlaves ();

      boolean masterFound = selectMasterAgent ();
      if (masterFound) {
         // get source mesh
         PolygonalMesh sourceMesh;
         ArrayList<PolygonalMesh> srcs = new ArrayList<PolygonalMesh> ();
         for (MeshModelAgent agent : masterRbAgent) {
            sourceMesh = (PolygonalMesh)agent.getMesh ();
            srcs.add (sourceMesh);
         }
         for (FemModelAgent agent : masterFemAgent) {
            sourceMesh = agent.regenerateSurfaceMeshAgent ();
            srcs.add (sourceMesh);
         }
         for (MeshModelAgent agent : masterExternalMeshAgents) {
            sourceMesh = (PolygonalMesh)agent.getMesh ();
            srcs.add (sourceMesh);
         }


         // get target mesh
         ArrayList<PolygonalMesh> tgts = new ArrayList<PolygonalMesh> ();
         String myPath = getSubjectPath ();
         String registrationPath = myPath  + "/registration";
         for (MeshModelAgent agent : masterRbAgent) {
            PolygonalMesh tgtMesh = null;
            String boneName = agent.getClient ().getName ();
            String meshPath = registrationPath + "/" + boneName + 
            "/" + boneName + "TargetMesh";
            tgtMesh = readMeshWithoutSuffix (meshPath);

            if (tgtMesh != null) tgts.add (tgtMesh);
            else throw new DataLoadException ("No target mesh data for Frank RigidBody " +
            boneName + "!");
         }
         for (FemModelAgent agent : masterFemAgent) {
            PolygonalMesh tgtMesh = null;
            String femName = agent.getClient ().getName ();
            String meshPath = registrationPath + "/" + femName + 
            "/" + femName + "TargetMesh";
            tgtMesh = readMeshWithoutSuffix (meshPath);

            if (tgtMesh != null) tgts.add (tgtMesh);
            else throw new DataLoadException ("No target mesh data for Frank soft-tissue " +
            femName + "!");
         }
         for (MeshModelAgent agent : masterExternalMeshAgents) {
            PolygonalMesh tgtMesh = null;
            String extName = agent.getClient().getName ();
            String meshPath = registrationPath + "/" + extName + 
            "/" + extName + "TargetMesh";
            tgtMesh = readMeshWithoutSuffix (meshPath);

            if (tgtMesh != null) tgts.add (tgtMesh);
            else throw new DataLoadException ("No target mesh data for external data " +
            extName + "!");
         }

         icp = new ICPManager ("ICP");
         Map<MeshBase, MeshBase> meshMap = new HashMap<MeshBase, MeshBase>();
         for (int i = 0; i < srcs.size (); i++) {
            meshMap.put (srcs.get (i), tgts.get (i));
         }
         icp.initialize (meshMap, null, null);
         addModel(icp);
         icp.renderSourceAndTargetMesh (this);
         myRegPanel = icp.createControlPanel (this);

         myRegPanel.addWidget (new JSeparator ());
         myRegPanel.addLabel ("Import Slaves");
         myRegPanel.addWidget (addRbSlaveButton);
         myRegPanel.addWidget (addFemSlaveButton);
         myRegPanel.addWidget (this, "selectRbComp");
         myRegPanel.addWidget (this, "selectFemComp");
         myRegPanel.addWidget (this, "selectICP");

         myRegPanel.addWidget (new JSeparator ());
         myRegPanel.addLabel ("Result");
         myRegPanel.addWidget (renderMasterAttachmentButton);
         myRegPanel.addWidget (updateMasterAttachmentButton);
         myRegPanel.addWidget (transformSlaveInBatchButton);
         myRegPanel.addWidget (saveMasterButton);
         myRegPanel.addWidget (saveSlaveButton);

         addControlPanel (myRegPanel);
         registrationGenerated = true;
         mechModel.setDynamicsEnabled (false);
         mechModel.getRenderProps ().setVisible (false);

      }
      else {
         System.err.println ("No registerable components been selected!");
         System.err.println ("Pick one or more RigidBodies or DeformableBodies please!");
      }
   }

   /**
    * remove and disposal registration <code>ControlPanel</code>;<p>
    * remove registration source mesh;<p>
    * remove registration target mesh; <p>
    * remove attachment 
    * remove registration probes; <p>
    * remove <code>ICPManager<code>s;
    */
   public void clearRegistrationManager () {

      if (getControlPanels ().contains (myRegPanel)) {
         removeControlPanel (myRegPanel);
      }

      if (myRegPanel != null) myRegPanel.dispose ();
      if (icp != null) {
         icp.disableRenderSourceAndTargetMesh (this);
         if (this.getOutputProbes ().contains (icp.getRegistrationErrorProbe ()))
            removeOutputProbe (icp.getRegistrationErrorProbe ());
         if (models ().contains (icp)) removeModel (icp);
         icp = null;
      }
      for (MeshModelAgent agent : masterRbAgent) {
         agent.disableIsolatedPointFrameAttachmentSlaveAgents (this);
      }
      for (FemModelAgent agent : masterFemAgent) {
         //TODO
      }
      for (MeshModelAgent agent : masterExternalMeshAgents) {
         //TODO
      }
      registrationGenerated = false;
      mechModel.setDynamicsEnabled (true);
      mechModel.getRenderProps ().setVisible (true);
   }

   public void clearSlaves () {
      HashSet <TransformableGeometry> slaves = new HashSet <TransformableGeometry>();
      slaves.addAll (icpSlaves.values ());

      for (TransformableGeometry slave : slaves) {
         if (renderables ().contains (slave))
            renderables ().remove (slave);
      }
      for (TransformableGeometry slave : slavesInBatch.values ()) {
         if (renderables ().contains (slave))
            renderables ().remove (slave);
      }
      icpSlaves.clear ();
      slavesInBatch.clear ();
   }

   /**
    * add rigid body slave
    */
   private void addRbSlave () {
      if (icp == null) {
         throw new ImproperStateException ("Registration managers not initialized");
      }

      if (frankRbs != FrankRigidBodyComponents.NULL) {
         String rbName = frankRbs.toString ();
         TransformableGeometry rb = rbs.get (rbName);
         MeshModelAgent agent = null;
         // make sure no repeated slaves
         if (icpSlaves.containsKey (rb)) {
            agent = (MeshModelAgent)icpSlaves.get (rb);
         }
         else if (slavesInBatch.containsKey (rb)) {
            agent = (MeshModelAgent)slavesInBatch.get (rb);
            slavesInBatch.remove (rb);
         }
         else {
            for (MeshModelAgent masterAgent : masterRbAgent) {
               if (masterAgent.getClient () == rb) {
                  agent = masterAgent;
                  break;
               }
            }
         }
         if (agent == null) {
            agent = new MeshModelAgent ((RigidBody)rb);
            addRenderable (agent);
         }
         agent.setName (((RigidBody)rb).getName () + "Agent");

         icpSlaves.put (rb, agent);

         if (!icp.containSlave (agent)) {
            icp.addSlave (agent);
            icp.assignSlaveInfo (agent);
            icp.addNFFDSlavePanel (icp.
               getSlaveInfo (agent),myRegPanel, this);
         }

      }
   }

   /**
    * add deformable slave
    */
   private void addFemSlave () {
      if (icp == null) {
         throw new ImproperStateException (
         "Registration managers not initialized");
      }

      if (frankFems != FrankFemComponents.NULL) {
         String femName = frankFems.toString ();
         TransformableGeometry fem = fems.get (femName);
         FemModelAgent agent = null;
         // make sure no repeated slaves
         if (icpSlaves.containsKey (fem)) {
            agent = (FemModelAgent)icpSlaves.get (fem);
         }
         else if (slavesInBatch.containsKey (fem)) {
            agent = (FemModelAgent)slavesInBatch.get (fem);
            slavesInBatch.remove (fem);
         }
         else {
            for (FemModelAgent masterAgent : masterFemAgent) {
               if (masterAgent.getClient () == fem) {
                  agent = masterAgent;
                  addRenderable (agent);
                  break;
               }
            }
         }
         if (agent == null) {
            agent = new FemModelAgent ((FemMuscleModel)fem);
            agent.regenerateSurfaceMeshAgent ();
            addRenderable (agent);
         }
         agent.setName (((FemMuscleModel)fem).getName () + "Agent");

         icpSlaves.put (fem, agent);

         if (!icp.containSlave (agent)) {
            icp.addSlave (agent);
            icp.assignSlaveInfo (agent);
            icp.addNFFDSlavePanel (icp.
               getSlaveInfo (agent), myRegPanel, this);
         }

      }
   }

   private HashMap <TransformableGeometry, TransformableGeometry> slavesInBatch = 
   new HashMap<TransformableGeometry, TransformableGeometry> ();

   //TODO: transformation order
   private void applyTransformToSlaves () {
      if (icp == null) {
         throw new ImproperStateException ("ICP managers not initialized");
      }
      for (TransformableGeometry slave : slavesInBatch.values ()) {
         if (renderables ().contains (slave))
            renderables ().remove (slave);
      }
      slavesInBatch.clear ();

      for (RigidBody rb : rbs) {
         if (rb.isSelected ()) {
            MeshModelAgent agent = new MeshModelAgent (rb);
            agent.setName (rb.getName ()+"BatchAgent");
            slavesInBatch.put (rb, agent);
         }
      }
      for (FemMuscleModel fem: fems) {
         if (fem.isSelected ()) {
            FemModelAgent agent = new FemModelAgent (fem);
            agent.setName (fem.getName ()+"BatchAgent");
            slavesInBatch.put (fem, agent);
         }
      }
      for (MeshModelAgent ext : externalMeshAgents) {
         if (ext.isSelected ()) {
            MeshModelAgent agent = new MeshModelAgent (ext);
            agent.setName (ext.getName () + "BatchAgent");
            slavesInBatch.put (ext, agent);
         }
      }
      if (slavesInBatch.size () == 0 ) {
         System.err.println ("Please select slaves in hierachy panel");
         return;
      }

      if (icp != null) {
         AffineModelTransformer affine = icp.getAffine ();
         for (TransformableGeometry slave : slavesInBatch.values ()) {
            if (slave instanceof MeshModelAgent) {
               MeshModelAgent agent = (MeshModelAgent)slave;
               affine.makeAccumulatedTransform (agent.getMesh ());
            }
            else {
               affine.makeAccumulatedTransform (slave);
            }
         }

         NFFDeformer ffd= icp.getFFD ();
         for (TransformableGeometry slave : slavesInBatch.values ()) {
            if (slave instanceof MeshModelAgent) {
               MeshModelAgent agent = (MeshModelAgent)slave;
               ffd.makeAccumulatedTransform (agent.getMesh ());
            }
            else{
               ffd.makeAccumulatedTransform (slave);
            }
         }
      }

      for (TransformableGeometry slave : slavesInBatch.values ()) {
         addRenderable ((RenderableComponent)slave);
      }
   }

   public void saveMasterResult () {
      String myPath = getSubjectPath ();
      String registrationPath = myPath  + "/registration";

      for (MeshModelAgent agent : masterRbAgent) {
         String boneName = agent.getClient ().getName ();
         String meshPath = registrationPath + "/" + boneName + 
         "/" + boneName + "ResultMesh";
         // write mesh
         PolygonalMesh meshToWrite = (PolygonalMesh)agent.getMesh ();
         writeMeshWithoutSuffix (meshToWrite, meshPath);
      }
      for (FemModelAgent agent : masterFemAgent) {
         String femName = agent.getClient ().getName ();
         String meshPath = registrationPath + "/" + femName + 
         "/" + femName + "ResultFem";
         // write volume mesh
         writeFemWithoutSuffix (agent, meshPath);
      }
      System.out.println ("Registration result writting for masters done!");
   }

   public void saveSlaveResult () {
      String myPath = getSubjectPath ();
      String registrationPath = myPath  + "/registration";

      ArrayList<TransformableGeometry> slaveAgents = 
      new ArrayList<TransformableGeometry> ();

      slaveAgents.addAll (icpSlaves.values ());
      slaveAgents.addAll (slavesInBatch.values ());
      for (TransformableGeometry slaveAgent: slaveAgents) {
         if (slaveAgent instanceof MeshModelAgent) {
            MeshModelAgent agent = (MeshModelAgent)slaveAgent;
            String boneName = agent.getClient ().getName ();
            String meshPath = registrationPath + "/" + boneName + 
            "/" + boneName + "ResultMesh";
            // write mesh
            PolygonalMesh meshToWrite = (PolygonalMesh)agent.getMesh ();
            writeMeshWithoutSuffix (meshToWrite, meshPath);
         }
         else if (slaveAgent instanceof FemModelAgent){
            FemModelAgent agent = (FemModelAgent) slaveAgent;
            String femName = agent.getClient ().getName ();
            String meshPath = registrationPath + "/" + femName + 
            "/" + femName + "ResultFem";
            // write volume mesh
            writeFemWithoutSuffix (agent, meshPath);
         }
      }
      System.out.println ("Registration result writting for slaves done!");
   }

   public void renderMasterMuscleAttachments () {
      for (MeshModelAgent agent : masterRbAgent) {
         agent.renderIsolatedPointFrameAttachmentSlaveAgents (this);
      }
      for (FemModelAgent agent : masterFemAgent) {
         //TODO
      }
   }

   public void updateMasterMuscleAttachments () {
      for (MeshModelAgent agent : masterRbAgent) {
         agent.updateIsolatedPointFrameAttachmentSlaveAgents ();
      }
      for (FemModelAgent agent : masterFemAgent) {
         agent.updateAttachmentPos ();
      }
   }

   public void passResultToSubject () {
      String path = getSubjectPath ();
      String resultPath = path + "/registration";
      for (RigidBody rb :  rbs) {
         String rbName = rb.getName ();
         String rbResultPath = resultPath + "/" + rbName;
         File rbResultDir = new File (rbResultPath);
         if (rbResultDir.exists () && rbResultDir.isDirectory ()) {
            String rbMeshPath = rbResultPath + "/" + rbName + "ResultMesh.obj";
            File rbResultFile = new File (rbMeshPath);
            if (rbResultFile.exists ()) {
               String rbPath = getSubjectRigidBodyPath (rb);
               File rbDir = new File (rbPath);
               if (!rbDir.exists ()) {
                  rbDir.mkdir ();
               }
               File rbMesh = new File (rbPath + "/" + rbName + "Mesh.obj");
               try {
                  Files.copy (rbResultFile, rbMesh);
                  System.out.println ("Data pass for " + rbName + " done!");
               }
               catch (IOException e) {
                  e.printStackTrace();
                  System.err.println ("Fail to transfer " + rbName + 
                  "'s result file!");
               }
            }
         }
      }

      for (FemModel3d fem :  fems) {
         String femName = fem.getName ();
         String femResultPath = resultPath + "/" + femName;
         File femResultDir = new File (femResultPath);
         if (femResultDir.exists () && femResultDir.isDirectory ()) {
            String femMeshPath = femResultPath + "/" + femName + "ResultFEM.vtk";
            File femResultFile = new File (femMeshPath);
            if (femResultFile.exists ()) {
               String femPath = getSubjectDeformableBodyPath (fem);
               File femDir = new File (femPath);
               if (!femDir.exists ()) {
                  femDir.mkdir ();
               }
               File femMesh = new File (femPath + "/" + femName + "FEM.vtk");
               try {
                  Files.copy (femResultFile, femMesh);
                  System.out.println ("Data pass for " + femName + " done!");
               }
               catch (IOException e) {
                  e.printStackTrace();
                  System.err.println ("Fail to transfer " + femName + 
                  "'s result file!");
               }
            }
         }
      }

      for (MeshModelAgent ext : externalMeshAgents) {
         String extName = ext.getName ();
         String extResultPath = resultPath + "/" + extName;
         File extResultDir = new File (extResultPath);
         if (extResultDir.exists () && extResultDir.isDirectory ()) {
            String extMeshPath = extResultPath + "/" + extName + "ResultMesh.obj";
            File extResultFile = new File (extMeshPath);
            if (extResultFile.exists ()) {
               String extPath = getSubjectPath () + "/" + extName;
               File extDir = new File (extPath);
               if (!extDir.exists ()) {
                  extDir.mkdir ();
               }
               File extMesh = new File (extPath + "/" + extName + "Mesh.obj");
               try {
                  Files.copy (extResultFile, extMesh);
                  System.out.println ("Data pass for " + extName + " done!");
               }
               catch (IOException e) {
                  e.printStackTrace();
                  System.err.println ("Fail to transfer " + extName + 
                  "'s result file!");
               }
            }
         }
      }
   }


   RenderableComponentList <RenderableComponent> targetDataAgents = 
   new RenderableComponentList<RenderableComponent> (RenderableComponent.class, 
   "Target Data", "TD");
   boolean targetDataRendered = false;

   public void renderRegistrationTargetData () {
      targetDataAgents.clear ();
      String path = getSubjectPath ();
      String resultPath = path + "/registration";
      ArrayList <RenderableComponent> list = new ArrayList<RenderableComponent> ();
      list.addAll (rbs);
      list.addAll (fems);
      for (RenderableComponent rp :  list) {
         String rpName = rp.getName ();
         String rpResultPath = resultPath + "/" + rpName;
         File rpResultDir = new File (rpResultPath);
         if (rpResultDir.exists () && rpResultDir.isDirectory ()) {
            String rpMeshPath = rpResultPath + "/" + rpName + "TargetMesh";
            PolygonalMesh mesh = readMeshWithoutSuffix (rpMeshPath);
            if (mesh != null) {
               MeshModelAgent agent = new MeshModelAgent(rp.getName ()+ "TargetMeshAgent");
               agent.represent (mesh);
               RenderProps.setFaceStyle (agent, FaceStyle.FRONT_AND_BACK);
               RenderProps.setFaceColor (agent, Color.CYAN);
               RenderProps.setShading (agent, Shading.SMOOTH);
               RenderProps.setDrawEdges (agent, false);
               targetDataAgents.add (agent);
            }
         }
      }

      for (MeshModelAgent srcagent : externalMeshAgents) {
         PolygonalMesh tgtMesh = null;
         String meshName = srcagent.getName ();
         String meshPath = getSubjectPath() + "/registration/" + meshName + 
         "/" + meshName + "TargetMesh";
         tgtMesh = readMeshWithoutSuffix (meshPath);
         MeshModelAgent agent = new MeshModelAgent(meshName+"TargetMeshAgent");
         if (tgtMesh != null) {
            agent.represent (tgtMesh);
            RenderProps.setFaceStyle (agent, FaceStyle.FRONT_AND_BACK);
            RenderProps.setFaceColor (agent, Color.CYAN);
            RenderProps.setShading (agent, Shading.SMOOTH);
            RenderProps.setDrawEdges (agent, false);
            targetDataAgents.add (agent);
         }
      }
      addRenderable(targetDataAgents);
      targetDataRendered = true;
   }

   public void disableRenderRegistrationTargetData () {
      removeRenderable(targetDataAgents);
      targetDataAgents.clear ();
      targetDataRendered = false;
   }


   public void getMuscleBundles (ArrayList<MuscleBundle> bundles) {

      for (FemMuscleModel fem : fems) {
         // TODO: remove this
         if (fem.getName () == "face") {
            continue;
         }
         FemMuscleModel client = (FemMuscleModel)fem;
         for (MuscleBundle mb : client.getMuscleBundles ()) {
            bundles.add (mb);
         }
      }

      for (MuscleBundle mus : externalMuscles) {
         bundles.add (mus);
      }

      for (MuscleBundle lig : ligaments) {
         bundles.add (lig);
      }
   }


   //TODO: remove slave
   //TODO: checkBox for slave 




   // -------------------------------------
   // implement action listener
   // -------------------------------------

   JButton addRbSlaveButton = new JButton ("Add RigidBody Slave");
   JButton addFemSlaveButton = new JButton ("Add Deformable Slave");
   JButton renderMasterAttachmentButton = new JButton ("Show Point Agent Attached to Masters");
   JButton updateMasterAttachmentButton = new JButton ("Update Point Agent Attached to Masters");
   JButton transformSlaveInBatchButton = new JButton ("Apply Transform To Choosen Slave");
   JButton saveMasterButton = new JButton ("Save Result of Masters");
   JButton saveSlaveButton = new JButton ("Save Result of Slaves");


   @Override
   public void actionPerformed(ActionEvent event) {
      if (event.getActionCommand ().equals ("LoadSubjectData")) {
         loadSubjectData();
      }
      else if (event.getActionCommand ().equals ("MakeSubjectModel")) {
         makeSubjectModel ();

      }
      else if (event.getActionCommand ().equals ("CreateRegistrationManager")) {
         try {
            generateRegistrationManager ();
         }
         catch (ModelGrowException e) {
            e.printStackTrace ();
            System.err.println ("Failed to make subject specific model");
         }
      }
      else if (event.getActionCommand ().equals ("RemoveRegistrationManager")) {
         clearRegistrationManager ();
         clearSlaves();
      }
      else if (event.getActionCommand ().equals ("PassResultToSubject")) {
         passResultToSubject ();
      }
      else if (event.getActionCommand ().equals ("RenderTargetMeshes")) {
         renderRegistrationTargetData ();
      }
      else if (event.getActionCommand ().equals ("RemoveTargetMeshes")) {
         disableRenderRegistrationTargetData ();
      }
      else if (event.getSource () == addRbSlaveButton) {
         addRbSlave ();
      }
      else if (event.getSource () == addFemSlaveButton) {
         addFemSlave ();
      }
      else if (event.getSource () == renderMasterAttachmentButton) {
         renderMasterMuscleAttachments ();
      }
      else if (event.getSource () == updateMasterAttachmentButton) {
         updateMasterMuscleAttachments ();
      }
      else if (event.getSource () == saveMasterButton) {
         saveMasterResult ();
      }
      else if (event.getSource () == saveSlaveButton) {
         saveSlaveResult ();
      }
      else if (event.getSource () == transformSlaveInBatchButton) {
         applyTransformToSlaves ();
      }
   }

   @Override
   public boolean getMenuItems(List<Object> items) {

      JMenuItem menuItem;
      String name;

      name = "LoadSubjectData";
      menuItem = makeMenuItem (name, "Load subject data");
      items.add (menuItem);

      name = "MakeSubjectModel";
      menuItem = makeMenuItem (name, "make subject-specific model, "
      + "should load subject data first");
      items.add (menuItem);

      if (registrationGenerated) {
         name = "RemoveRegistrationManager";
         menuItem = makeMenuItem (name, "remove registration manager");
         items.add (menuItem);
      }
      else {
         name = "CreateRegistrationManager";
         menuItem = makeMenuItem (name, "create registration manager");
         items.add (menuItem);
      }

      name = "PassResultToSubject";
      menuItem = makeMenuItem (name, "Pass registsration result files to subject folder");
      items.add (menuItem);

      if (targetDataRendered) {
         name = "RemoveTargetMeshes";
         menuItem = makeMenuItem (name, "Disable all target meshes rendering");
         items.add (menuItem);
      }
      else {
         name = "RenderTargetMeshes";
         menuItem = makeMenuItem (name, "Render all target meshes in "
         + "registration directroy of the subject");
         items.add (menuItem);
      }


      return true;
   }   

   protected JMenuItem makeMenuItem (String cmd, String toolTip) {
      JMenuItem item = new JMenuItem(cmd);
      item.addActionListener(this);
      item.setActionCommand(cmd);
      if (toolTip != null && !toolTip.equals ("")) {
         item.setToolTipText (toolTip);
      }
      return item;
   }


   /**
    * The subject name return must be consistent with
    * the name of its data folder; The method will be 
    * called during <code>SubjectModel</code> building; 
    * The name returned will be set as the name of the
    * subclass model, which will be used to retrieve 
    * subject data for this subject;
    * @return subject name
    */
   protected abstract String getSubjectName ();

   //TODO
   protected abstract void getExternalDataNames (ArrayList <String> extSrcNames);


   public static PolygonalMesh readMeshWithoutSuffix (
      String absolutePathWithoutSuffix) {
      String [] suffixes = new String [4];
      suffixes [0] = ".ply";
      suffixes [1] = ".obj";
      suffixes [2] = ".stl";
      suffixes [3] = ".vtk";
      boolean findMesh = false;
      File meshFile = null;
      String suffix = null;
      for (int i = 0; i < suffixes.length; i++) {
         suffix = suffixes[i];
         meshFile = new File (absolutePathWithoutSuffix + suffix);
         if (meshFile.exists () && meshFile.isFile ()) {
            findMesh = true;
            break;
         }
      }

      PolygonalMesh meshData = null;
      if (findMesh) {
         try {
            if (suffix == suffixes[3]) {
               meshData = VtkAsciiReader.read(meshFile);
            }
            else {
               meshData = (PolygonalMesh)GenericMeshReader.readMesh (meshFile);
            }
         }
         catch (IOException e) {
            e.printStackTrace();
            System.err.println(
            "Failed to read mesh!");
         }
      }
      else {
         System.err.println ("Mesh data: " + absolutePathWithoutSuffix + 
         " not found!");
      }
      return meshData;
   }

   public static void writeMeshWithoutSuffix(PolygonalMesh meshToWrite, 
      String absolutePathWithoutSuffix) {

      boolean fixed = meshToWrite.isFixed ();
      if (fixed) {
         meshToWrite.setFixed (false);
      }
      if (!meshToWrite.meshToWorldIsIdentity ()) {
         meshToWrite.transform (meshToWrite.getMeshToWorld ());
      }
      try {
         File temF = new File(absolutePathWithoutSuffix+".obj");
         meshToWrite.write (temF, null);
      } 
      catch (IOException e) {
         e.printStackTrace ();
         System.err.println ("Failed to write mesh");
      }

      if (!meshToWrite.meshToWorldIsIdentity ()) {
         meshToWrite.inverseTransform (meshToWrite.getMeshToWorld ());
      }
      if (fixed) {
         meshToWrite.setFixed (true);
      }
   }

   public static boolean readFemWithoutSuffix (FemModel3d femToLoad, 
      String absolutePathWithoutSuffix) {
      String suffix = ".vtk";
      File meshFile = new File (absolutePathWithoutSuffix + suffix);
      if (meshFile.exists () && meshFile.isFile ()) {
         String name = femToLoad.getName ();
         VtkInputOutput.readUnstructuredMesh_volume(femToLoad, 
            absolutePathWithoutSuffix + suffix);
         if (name != null) {
            femToLoad.setName (name);
         }
         return true;
      }
      return false;
   }

   public static void writeFemWithoutSuffix (FemModel3d femToWrite, 
      String absolutePathWithoutSuffix) {
      VTK_IO.writeVTK(absolutePathWithoutSuffix + ".vtk", femToWrite);
   }

   protected MatrixNd writeBoneSampledMarkers (
      String name, String filePath, String outputFilePath, int num, double den) {
      PolygonalMesh mesh = SubjectModel.readMeshWithoutSuffix (filePath);

      RigidBody bone = rbs.get (name);
      if (bone == null) {
         return null;
      }

      // compute area
      double area = 0;
      for (int i  = 0; i < mesh.numFaces (); i++) {
         Face face = mesh.getFace (i);
         area += face.computeArea ();
      }

      boolean [] marks = null;
      int numVts = 0;

      if (num >= mesh.numVertices () && den < 0) {
         marks = new boolean [mesh.numVertices ()];
         numVts = mesh.numVertices ();
      }
      else if (num >= mesh.numVertices ()) {
         numVts = (int) (den * area);
         System.out.println (numVts);
         if (numVts < (mesh.numVertices ()/2.0)) {
            marks= UniformMeshFeatureSubsampler.createRandomBinaryIndices (
               numVts, mesh.numVertices ());
         }
         else {
            marks= UniformMeshFeatureSubsampler.createRandomBinaryIndices (
               mesh.numVertices () - numVts, mesh.numVertices ());
         }
      }
      else {
         numVts = num;
         if (numVts < (mesh.numVertices ()/2.0)) {
            marks= UniformMeshFeatureSubsampler.createRandomBinaryIndices (
               numVts, mesh.numVertices ());
         }
         else {
            marks= UniformMeshFeatureSubsampler.createRandomBinaryIndices (
               mesh.numVertices () - numVts, mesh.numVertices ());
         }
      }

      MatrixNd Mat = new MatrixNd (0, 3);
      int idx = -1;
      int rowIdx = 0;
      for (Vertex3d vtx : mesh.getVertices ()) {
         idx++;
         if (numVts < (mesh.numVertices ()/2.0)) {
            if (!marks[idx]) {
               continue;
            }
         }
         else {
            if (marks[idx]) {
               continue;
            }
         }

         Mat.setSize (Mat.rowSize () + 1, 3);
         Mat.setRow (rowIdx, vtx.getWorldPoint ());
         rowIdx ++;
      }

      if (outputFilePath != null) {
         try {
            ReadWrite.writeMatrixToFile (Mat, outputFilePath);
         }
         catch (IOException e) {
            e.printStackTrace();
         }
      }

      return Mat;
   }

   protected MatrixNd writeFemSampledMarkers (
      String name, String filePath, String outputFilePath, int num, double den){
      PolygonalMesh mesh = SubjectModel.readMeshWithoutSuffix (
         filePath);

      FemModel3d fem = fems.get (name);

      double th = 0.001;

      // find valid vertices
      ArrayList<Vertex3d> vts = new ArrayList <Vertex3d> ();
      HashMap <Vertex3d, Point3d> map = new HashMap <Vertex3d, Point3d>();
      for (int i = 0; i< mesh.numVertices (); i++) {
         Vertex3d vtx = mesh.getVertex (i);

         Point3d pos = new Point3d ();
         vtx.getWorldPoint (pos);
         Point3d loc = new Point3d ();
         Point3d result = new Point3d ();

         FemElement3d ele = fem.findContainingElement (pos);
         if (ele == null) {
            ele = fem.findNearestSurfaceElement (loc, pos);
            if (ele == null) {
               continue;
            }
            if (loc.distance (pos) > th) {
               continue;
            }
            else{
               vts.add (vtx);
               Point3d results = new Point3d ();
               results.add (loc, pos);
               map.put (vtx, loc);
            }
         }
         else {
            vts.add (vtx);
            map.put (vtx, pos);
         }
      }

      // compute area
      double area = 0;
      for (int i  = 0; i < mesh.numFaces (); i++) {
         Face face = mesh.getFace (i);
         boolean flag = true;
         for (int j = 0; j < face.numVertices (); j++) {
            Vertex3d vtx = face.getVertex (j);
            if (!vts.contains (vtx)) {
               flag = false;
               break;
            }
         }

         if (flag) {
            area += face.computeArea ();
         }
      }

      boolean [] marks = null;
      int numVts = 0;

      if (num >= vts.size () && den < 0) {
         marks = new boolean [vts.size ()];
         numVts = vts.size ();
      }
      else if (num >= vts.size ()) {
         numVts = (int) (den * area);
         System.out.println (numVts);
         if (numVts > vts.size ()) {
            numVts = vts.size ();
         }
         System.out.println (numVts);
         if (numVts < (vts.size () / 2.0)) {
            marks= UniformMeshFeatureSubsampler.createRandomBinaryIndices (
               numVts, vts.size ());
         }
         else {
            System.out.println ("start");
            marks= UniformMeshFeatureSubsampler.createRandomBinaryIndices (
               vts.size () - numVts, vts.size ());
            System.out.println ("finish");
         }
      }
      else {
         numVts = num;
         if (num < (vts.size () / 2.0)) {
            marks= UniformMeshFeatureSubsampler.createRandomBinaryIndices (
               num, vts.size ());
         }
         else {
            marks= UniformMeshFeatureSubsampler.createRandomBinaryIndices (
               vts.size () - num, vts.size ());
         }
      }

      MatrixNd Mat = new MatrixNd (0, 3);
      int idx = -1;
      int rowIdx = 0;
      for (Vertex3d vtx : vts) {
         idx++;
         if (numVts < (vts.size () / 2.0)) {
            if (!marks[idx]) {
               continue;
            }
         }
         else {
            if (marks[idx]) {
               continue;
            }
         }

         Mat.setSize (Mat.rowSize () + 1, 3);
         Mat.setRow (rowIdx, map.get (vtx));
         rowIdx ++;
      }

      if (outputFilePath != null) {
         try {
            ReadWrite.writeMatrixToFile (Mat, outputFilePath);
         }
         catch (IOException e) {
            e.printStackTrace();
         }
      }


      return Mat;
   }

   public void addAirway () {
      PolygonalMesh mesh = SubjectModel.readMeshWithoutSuffix (
         SUBJECT_DATA_ROOT_PATH + "/template/geometry/airway/airwayMesh");

      if (mesh == null) {
         System.err.println ("Airway mesh is not found!");
      }
      airway = new SkinMeshBody(mesh);
      airway.setName("subjectAirway");
      mechModel.addMeshBody(airway);

      for (FemModel3d fem : fems) {
         airway.addFemModel (fem);
      }
      airway.addFrame (rbs.get ("jaw"));
      airway.addFrame (rbs.get ("maxilla"));
      airway.computeWeights();

      RenderProps props = airway.getRenderProps ();
      props.setFaceStyle(Renderer.FaceStyle.FRONT_AND_BACK);
      props.setFaceColor(new Color(0.2f, 0.4f, 1.0f) );
      props.setShading(Renderer.Shading.SMOOTH);
      props.setVisible (false);
   }


   public ArrayList<NumericInputProbe> forwardTestMuscleInputProbe (String dataFolderPath) {
      File dir = new File (dataFolderPath);
      if (!dir.exists ()) {
         System.err.println (dataFolderPath);
         throw new IllegalArgumentException (
         "path not found!");
      }
      if (!dir.isDirectory ()) {
         throw new IllegalArgumentException (
         "path not a directory!");
      }

      String [] fileNames = dir.list ();
      ArrayList<String> profileNames = new ArrayList<String> ();
      ArrayList<String> shortNames = new ArrayList<String> ();
      for (String fileName : fileNames) {
         if (fileName.endsWith (".muscleForward")) {
            String fN = dataFolderPath + "/" + fileName;
            profileNames.add (fN);
            shortNames.add (fileName);
         }
      }

      ArrayList<NumericInputProbe> inputs = new ArrayList<NumericInputProbe> ();

      // create exciter 2 index map
      HashMap <MuscleExciter, Integer> map = new HashMap <MuscleExciter, Integer> ();
      int exIdx = 0;;
      for (ComponentList list : exciters) {
         ComponentList<MuscleExciter> exs = (ComponentList<MuscleExciter>)list;
         for (MuscleExciter ex : exs) {
            map.put (ex, new Integer (exIdx++));
         }
      }

      int idx = -1;
      for (String path : profileNames) {
         ArrayList<String []> profileData = readProfile (path);
         idx++;

         // parse start
         String [] lineSplit;
         lineSplit = profileData.get (0);
         double [] timeKnots = new double [lineSplit.length];

         for (int i = 0; i < lineSplit.length; i++) {
            timeKnots[i] = Double.parseDouble (lineSplit[i]);
         }
         NumericInputProbe input = new NumericInputProbe (
            this, "exciterActivations", timeKnots[0], timeKnots[timeKnots.length-1]);
         input.setName (shortNames.get (idx).replaceAll (".muscleForward", ""));

         for (int t = 0; t < timeKnots.length; t++) {
            double time = timeKnots [t];
            double [] data = new double [map.size ()];
            for (int i = 1; i < profileData.size (); i++) {
               lineSplit = profileData.get (i);
               String cn = lineSplit[0] + "Exciters";
               ComponentList<MuscleExciter> exs = exciters.get (cn);
               if (exs == null) {
                  throw new NullPointerException (
                     cn + " not found!");
               }
               lineSplit[1] = lineSplit[1].trim ();
               MuscleExciter ex = exs.get (lineSplit[1]);
               if (ex == null) {
                  throw new NullPointerException (
                     lineSplit[1] + " not found!");
               }
               data [map.get (ex)] = Double.parseDouble (lineSplit [t + 2]);
            }

            input.addData (timeKnots[t], data);
         }

         inputs.add (input);
      }

      return inputs;
   }

   public static ArrayList<String []> readProfile(String filename)
   {   
      ArrayList<String []> DataList = new ArrayList<String []> ();

      try 
      {
         BufferedReader file = new BufferedReader(new FileReader(filename));
         String inLine = null;
         String[] inSplit = null;

         while (file.ready() == true)
         {
            inLine = file.readLine();
            inLine = inLine.trim();
            if (inLine.isEmpty() == true)
            {
               continue; // ignore
            }
            else if (inLine.startsWith("#") == true) {
               continue;   // comments are ignored
            }
            else 
            {
               inSplit = inLine.split(",");
               for (String str :  inSplit) {
                  str.trim ();
               }
               DataList.add (inSplit);
            }
         }
         file.close();
      }
      catch(Exception e) 
      {
         e.printStackTrace();
      }
      return DataList;
   }

   public void attach (DriverInterface driver) {
      super.attach (driver);
      ArtisynthPath.setWorkingDir (new File (
         getSubjectPath ()));
   }


   public static PolygonalMesh loadPNGs (String fileName, String name) {

      String scp = fileName.replaceAll (".png", "");
      scp = scp + "_PNGScript.txt";
      ArrayList<String[]> rawData = ReadWrite.readScript (scp);

      for (String [] inline : rawData) {
         for (String raw : inline) {
            raw.replaceAll (" ", "");
         }
      }

      //  voxel size
      double xs = Double.parseDouble (rawData.get (0)[0]);
      double ys = Double.parseDouble (rawData.get (0)[1]);

      // voxel number
      int xn = Integer.parseInt (rawData.get (1)[0]);
      int yn = Integer.parseInt (rawData.get (1)[1]);

      // transformation 
      AffineTransform3d X = new AffineTransform3d ();
      MatrixNd rMat=  new MatrixNd (4, 4);
      for (int i = 0; i < 4; i++) {
         for (int j = 0; j < 4; j++) {
            int line = i + 2;
            rMat.set (i, j, Double.parseDouble (rawData.get (line)[j]));
         }
      }
      X.set (rMat);

      // mesh
      PolygonalMesh mesh;
      mesh = MeshFactory.createPlane (xn * xs, yn * ys, xn, yn);
      mesh.setName (name);
      RenderProps.setSpecular (mesh, Color.BLACK); 
      RenderProps.setShading (mesh, Shading.SMOOTH); 
      RenderProps.setFaceColor (mesh, Color.WHITE);
      RenderProps.setFaceStyle (mesh, FaceStyle.FRONT_AND_BACK);

      // png
      ColorMapProps cprops = new ColorMapProps();
      cprops.setEnabled (true);
      cprops.setSpecularColoring (false);
      cprops.setFileName (fileName);
      RenderProps.setColorMap (mesh, cprops);
      RenderProps.setColorMapEnabled (mesh, true);

      // transformation
      mesh.transform (X);

      return mesh;
   }

}
