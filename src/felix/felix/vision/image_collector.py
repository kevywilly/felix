import os
import time
from pathlib import Path
import cv2
from felix.config.settings import settings
from uuid import uuid4

class ImageCollector:
    def __init__(self):
        self.counts = {}
        self._make_folders()
        self._generate_counts()
        

    def category_path(self, category: str) -> str:
        return os.path.join(settings.Training.training_data_path, category.replace(" ", "_"))

    def get_count(self, category: str) -> int:
        value = len(os.listdir(self.category_path(category)))
        self.counts[category] = value
        return value

    def _generate_counts(self):
        for category in settings.Training.categories:
            self.get_count(category)

    def _make_folders(self):

        try:
            os.makedirs(settings.Training.obstacle_path)
        except FileExistsError:
            pass

        try:
            os.makedirs(settings.Training.navigation_path)
        except FileExistsError:
            pass

        try:
            os.makedirs(settings.Training.training_data_path)
        except FileExistsError:
            pass

        for category in settings.Training.categories:
            
            try:
                os.makedirs(self.category_path(category))
            except FileExistsError:
                pass
            except Exception as ex:
                print(ex)
                raise ex
                
    def get_categories(self):
        return [{"name": k, "count": v} for k,v in self.counts.items()]

    def save_image(self, image, path, filename) -> bool:
        save_path = os.path.join(path,filename)
        with open(save_path, 'wb') as f:
            print(f"writing image to {save_path}")
            try:
                f.write(image)
            except Exception as ex:
                print(ex)
                return False
        
        return save_path

    def _jpeg_timestamp(self) -> bool:
        return f"{str(time.time()).replace('.','-')}.jpg"
    
    def save_obstacle_image(self, image) -> bool:
        return self.save_image(image, settings.Training.data_root, self._jpeg_timestamp())
    

    def save_navigation_image(self, x: int, y:int, width:int, height: int, image) -> str:
        name = 'xy_%03d_%03d_%03d_%03d_%s.jpg' % (x, y, width, height, uuid4())
        return self.save_image(
            image=image,
            path=settings.Training.navigation_path,
            filename=name
        )
        
    
    def collect(self, category: str, image) -> int:
        if category in settings.Training.categories:
            pth = self.save_image(image, self.category_path, self._jpeg_timestamp())
            return self.get_count(category)

        return -1

    def get_images(self, category):
        paths = sorted(Path(self.category_path(category)).iterdir(), key=os.path.getctime)
        return [p.name for p in paths]

    def load_image(self, category, name):
        im = cv2.imread(os.path.join(self.category_path(category), name), cv2.IMREAD_ANYCOLOR)
        _, im_bytes_np = cv2.imencode('.jpeg', im)

        return im_bytes_np.tobytes()

    def delete_image(self, category, name):
        try:
            os.remove(os.path.join(self.category_path(category), name))
            self._generate_counts()
        except:
            pass
            

        return True