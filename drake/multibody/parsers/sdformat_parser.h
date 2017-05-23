#pragma once

#include <memory>
#include <string>

#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/package_map.h"
#include "drake/multibody/parsers/parser_common.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace parsers {
namespace sdformat {

/**
 * Adds the model or models defined within an SDF file to a rigid body tree.
 * One instance of each model is added. The models in the SDF are assumed to be
 * described in the world frame.
 *
 * This method can only be used with SDF models that either (1) do not use
 * `package://` to reference modeling resources like mesh files, or (2)
 * only reference packages that are defined up the directory tree relative to
 * @p filename. SDF files that contain `package://` references to do not
 * meet these requirements should instead use
 * AddModelInstancesFromSdfFileToWorldSearchingInRosPackages().
 *
 * @param[in] filename The name of the SDF file containing the model to be
 * added.
 *
 * @param[in] floating_base_type The type of joint that connects the model's
 * root to the existing rigid body tree.
 *
 * @param[out] tree The rigid body tree to which to add the model.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
ModelInstanceIdTable
AddModelInstancesFromSdfFileToWorld(
    const std::string& filename,
    const drake::multibody::joints::FloatingBaseType floating_base_type,
    RigidBodyTree<double>* tree);

/**
 * Adds the model or models defined within an SDF file to a rigid body tree.
 * One instance of each model is added.
 *
 * This method can only be used with SDF models that either (1) do not use
 * `package://` to reference modeling resources like mesh files, or (2)
 * only reference packages that are defined up the directory tree relative to
 * @p filename. SDF files that contain `package://` references to do not
 * meet these requirements should instead use
 * AddModelInstancesFromSdfFileSearchingInRosPackages().
 *
 * @param[in] filename The name of the SDF file containing the model to be
 * added.
 *
 * @param[in] floating_base_type The type of joint that connects the model's
 * root to the existing rigid body tree.
 *
 * @param[in] weld_to_frame The frame to which to connect the new model.
 *
 * @param[out] tree The rigid body tree to which to add the model.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
ModelInstanceIdTable AddModelInstancesFromSdfFile(
    const std::string& filename,
    const drake::multibody::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    RigidBodyTree<double>* tree);

/**
 * Adds the model or models defined within an SDF description to @p tree. One
 * instance of each model is added.
 *
 * This method can only be used with SDF models that either (1) do not use
 * `package://` to reference modeling resources like mesh files, or (2)
 * only reference packages that are defined up the directory tree relative to
 * @p filename. SDF files that contain `package://` references to do not
 * meet these requirements should instead use
 * AddModelInstancesFromSdfStringSearchingInRosPackages().
 *
 * @param[in] sdf_string The SDF description of one or more models.
 *
 * @param[in] floating_base_type The type of joint that connects the model's
 * root to the existing rigid body tree.
 *
 * @param[in] weld_to_frame The frame to which to connect the new model.
 *
 * @param[out] tree The rigid body tree to which to add the model.
 *
 * @return A table mapping the names of the models whose instances were just
 * added to the `RigidBodyTree` to their instance IDs, which are unique within
 * the `RigidBodyTree`.
 */
ModelInstanceIdTable AddModelInstancesFromSdfString(
    const std::string& sdf_string,
    const drake::multibody::joints::FloatingBaseType floating_base_type,
    std::shared_ptr<RigidBodyFrame<double>> weld_to_frame,
    RigidBodyTree<double>* tree);


}  // namespace sdformat
}  // namespace parsers
}  // namespace drake
