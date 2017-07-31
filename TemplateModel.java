package artisynth.models.subjectFrank;

import java.util.ArrayList;

import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.Collidable.Collidability;
import artisynth.models.frank2.GenericModel;
import artisynth.models.frank2.frankUtilities.MeshEditor;
import maspack.geometry.PolygonalMesh;

public class TemplateModel extends SubjectModel{

   @Override
   protected String getSubjectName () {
      return "template";
   }

   @Override
   protected void getExternalDataNames (ArrayList<String> extSrcNames) {
   }
   
   public void build (String [] args) {
      super.build (args);
      defineCollision ();
   }

   protected void defineCollision () {
      mechModel.setCollisionBehavior(tongue, softPalate, true);
      mechModel.setCollisionBehavior(tongue, larynx, true);
      mechModel.setCollisionBehavior(tongue, rbs.get ("jaw"), true);
      mechModel.setCollisionBehavior(tongue, rbs.get ("maxilla"), true);
      //face.addMesh(GenericModel.loadGeometry_VTK(geometryDir + "face_collisionSurf_upperLip_reduced2.vtk"));
      //face.addMesh(GenericModel.loadGeometry_VTK(geometryDir + "face_collisionSurf_lowerLip_reduced.vtk"));
      //mechModel.setCollisionBehavior(face, face, true);
      mechModel.setCollisionBehavior(face, rbs.get ("maxilla"), true);
      mechModel.setCollisionBehavior(face, rbs.get ("jaw"), true);
      
      PolygonalMesh palateColl = GenericModel.loadGeometry(geometryDir, "palate_collisionSurf_velum_fine.ply"); // _02
      palateColl.triangulate();
      MeshEditor.snapSurfaceToSurface(palateColl, softPalate.getSurfaceMesh(), 0.01);
      FemMeshComp palateUvula = softPalate.addMesh(palateColl);
      palateUvula.setCollidable(Collidability.EXTERNAL);
      mechModel.setCollisionBehavior(palateUvula, pharynx, true);
      
      mechModel.getCollisionManager().setCompliance (1e-5);
      mechModel.getCollisionManager().setDamping(100.0);
   }
   
   protected ArrayList<FemNode3d> getFaceThyroidNodes (boolean surfaceOnly) {
      ArrayList<FemNode3d> nodesToAttach;
      if (surfaceOnly == true)
         nodesToAttach = GenericModel.findNodesNearSurface(
            face, rbs.get ("thyroid").getMesh (), 1E-3, true);
      else
         nodesToAttach = GenericModel.findNodesNearSurface(
            face, rbs.get ("thyroid").getMesh (), 1E-3, false);

      return nodesToAttach;
   }
   
   protected void attachFaceToThyroid (ArrayList<FemNode3d> nodeToAttach) {
      GenericModel.attachFemNodesToRigidBody(
         face, nodeToAttach, rbs.get ("thyroid"), mechModel);
   }
   
   protected void attachPharynxToThyroid () {
      GenericModel.attachFemToRigidBody(pharynx, rbs.get ("thyroid"), 0.001, true, mechModel);
   }
  
}
