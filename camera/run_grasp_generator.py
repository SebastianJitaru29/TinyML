from inference.grasp_generator import GraspGenerator

if __name__ == '__main__':
    generator = GraspGenerator(
        saved_model_path='trained-models/cornell-randsplit-rgbd-grconvnet3-drop1-ch16/epoch_17_iou_0.96',
        visualize=True
    )
    generator.load_model()
    generator.run()
