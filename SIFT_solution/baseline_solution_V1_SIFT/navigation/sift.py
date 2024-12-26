
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import pickle
from sklearn.cluster import KMeans
from sklearn.neighbors import BallTree
from sklearn.decomposition import PCA

class Exploration:
    def __init__(self, dir, targ_dir, num_clusters=128, vlad_dim=128):
        # Initialize class variables
        self.save_dir = dir  # Directory to save images to
        self.targ_dir = targ_dir
        # self.sift = cv2.xfeatures2d.SIFT_create()
        self.sift = cv2.SIFT_create()

        self.num_clusters = num_clusters  # Number of clusters for KMeans (codebook size)
        
        # Try to load the codebook, if not available, generate it
        current_path = os.getcwd()
        codebook_path = current_path + "/navigation/codebook_new.pkl"
        print(codebook_path)
        print("current path: ", current_path)

        self.pca = PCA(n_components=vlad_dim)
        
        if os.path.exists(codebook_path):
            with open(codebook_path, "rb") as f:
                self.codebook = pickle.load(f)
            print("Codebook loaded from file.")
        else:
            print("Codebook not found. Generating a new codebook.")
            self.codebook = self.generate_codebook()
            # Save the generated codebook for future use
            with open(codebook_path, "wb") as f:
                pickle.dump(self.codebook, f)
            print(f"Codebook saved to {codebook_path}.")
        
        # Initialize database for storing VLAD descriptors
        self.database = []
        self.tree = None

    def fit_pca(self):
        if self.database is None or len(self.database) == 0:
            print("Database is empty. Cannot fit PCA.")
            return

        # 训练 PCA 模型
        self.pca.fit(self.database)

        # 转换数据库描述子
        self.database = self.pca.transform(self.database)


    def display_img_from_id(self, id):
        """
        Display image from database based on its ID using OpenCV
        """
        path = self.save_dir + str(id).zfill(8) + ".jpg"
        img = cv2.imread(path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        plt.imshow(img)
        plt.axis('off')  # Turn off axis
        plt.show()

    def display_img_from_id_comparison(self, id, target_id):
        """
        Display image from database based on its ID using OpenCV
        """
        path = self.save_dir + str(id).zfill(8) + ".jpg"
        print(path)
        img_result = cv2.imread(path)
        img_result = cv2.cvtColor(img_result, cv2.COLOR_BGR2RGB)

        img_target = self.get_target_images()[target_id]
        img_target = cv2.cvtColor(img_target, cv2.COLOR_BGR2RGB)
        fig, (ax1, ax2) = plt.subplots(1, 2)

        ax1.imshow(img_result)
        ax1.set_title('Match Found')
        ax1.axis('off')  # Hide axis

        ax2.imshow(img_target)
        ax2.set_title('Target Image')
        ax2.axis('off')  # Hide axis

        plt.show()

    def get_target_images(self):
        img = []
        for filename in sorted(os.listdir(self.targ_dir)):
            if filename.endswith(".jpg"):
                path = os.path.join(self.targ_dir, filename)
                img.append(cv2.imread(path))
        return img
        
    def compute_sift_features(self):
        """
        Compute SIFT features for images in the data directory
        """
        sift_descriptors = list()
        length = len(os.listdir(self.save_dir))
        for i in range(length - 1):
            path = str(i).zfill(8) + ".jpg"
            img = cv2.imread(os.path.join(self.save_dir, path))
            _, des = self.sift.detectAndCompute(img, None)
            if des is not None:
                # sift_descriptors.extend(des)
                # L1 正规化
                des = des / (np.sum(np.abs(des), axis=1, keepdims=True) + 1e-7)
                # 开平方
                des = np.sqrt(des)
                sift_descriptors.extend(des)
        return np.asarray(sift_descriptors)

    def generate_codebook(self):
        """
        Generate a KMeans codebook by clustering SIFT descriptors
        """
        # Compute SIFT features for all images
        sift_descriptors = self.compute_sift_features()
        print(f"Number of SIFT descriptors: {len(sift_descriptors)}")

        # Perform KMeans clustering to generate the codebook
        kmeans = KMeans(n_clusters=self.num_clusters, random_state=0)
        kmeans.fit(sift_descriptors)

        print("Codebook generated using KMeans.")
        return kmeans

    def get_VLAD(self, img):
        """
        Compute VLAD (Vector of Locally Aggregated Descriptors) descriptor for a given image
        """
        _, des = self.sift.detectAndCompute(img, None)

        if des is None or self.codebook is None:
            print("No SIFT descriptors or codebook available. Cannot compute VLAD.")
            return None
        
        pred_labels = self.codebook.predict(des)
        centroids = self.codebook.cluster_centers_
        k = self.codebook.n_clusters
        VLAD_feature = np.zeros([k, des.shape[1]])

        for i in range(k):
            if np.sum(pred_labels == i) > 0:
                VLAD_feature[i] = np.sum(des[pred_labels == i, :] - centroids[i], axis=0)
        VLAD_feature = VLAD_feature.flatten()

        # Power normalization and L2 normalization
        VLAD_feature = np.sign(VLAD_feature) * np.sqrt(np.abs(VLAD_feature))
        VLAD_feature = VLAD_feature / np.linalg.norm(VLAD_feature)

        return VLAD_feature

    
    def get_neighbor(self, img):
        """
        Find the nearest neighbor in the database based on VLAD descriptor
        """
        q_VLAD = self.get_VLAD(img)
        
        if q_VLAD is None:
            return None
        
        q_VLAD = self.pca.transform(q_VLAD.reshape(1, -1))

        
        _, index = self.tree.query(q_VLAD, 1)
        return index[0][0]

    def pre_nav_compute(self, target_id):
        """
        Build BallTree for nearest neighbor search and find the goal ID
        """
        # tree = BallTree(self.database, leaf_size=40)
        # self.tree = tree
        targets = self.get_target_images()
        index = self.get_neighbor(targets[target_id])
        self.goal = index
        print(f'Goal ID: {self.goal}')

        return self.goal

    def run(self):
        vlad_descriptors = []

        for filename in sorted(os.listdir(self.save_dir)):
            if filename.endswith(".jpg"):
                target_path = os.path.join(self.save_dir, filename)
                img = cv2.imread(target_path)
                VLAD = self.get_VLAD(img)
                if VLAD is not None:
                    vlad_descriptors.append(VLAD)
        self.database = np.vstack(vlad_descriptors)

        # 训练 PCA 并转换描述子
        self.fit_pca()

        # 构建 BallTree
        self.tree = BallTree(self.database, leaf_size=40)

current_path = os.getcwd()
exploration = Exploration(current_path + "/baseline_solution_V1_SIFT/dataset/images/demo_2/",
                          current_path + "/baseline_solution_V1_SIFT/test/F24Target.jpg",
                          num_clusters=256)
exploration.run()

target_ID = 1
for i in range (0,5):
    target_ID = i
    ID = exploration.pre_nav_compute(target_ID)
    exploration.display_img_from_id_comparison(ID, target_ID)
