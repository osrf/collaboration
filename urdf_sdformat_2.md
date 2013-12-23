# Keep notes of urdfdom / sdformat 2.0 merging needs


 * `<gazebo>` extensions in urdf don't map directly to sdf (urdf uses camel cased variables, `provideFeedback` vs. `provide_feedback`).
 
 * Idea: should we support two different types of model definition in SDF? (tree for URDF, graph for sdformat). This would be similar to how it works now, but maybe rename some tags (`<robot>` -> `<tree>`, `<model>` -> `<graph>`) to be more explicit 
