# Observation images

New runs freeze `observation_image_profile: jpeg_896x560_q85` in their harness. Every observation camera produces an 896 × 560 JPEG at quality 85. The public task describes the observation format and resolution. Video recording has its own resolution and is unchanged.

A run with no profile uses the legacy `png_1280x800` profile. To prepare a new run with PNG observations, set that explicit profile in its harness configuration. Readers accept both JPEG and PNG; public reference assets keep their existing formats. A previously prepared run retains its frozen inputs and source; use the matching frozen version rather than changing its recorded configuration or assuming it can be resumed with different code.

When a published camera calibration specifies an image size, a run-specific copy is created with the observation resolution while the original published asset remains intact. Image generation and storage remain in their existing transaction scope.
