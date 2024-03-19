import os
from pathlib import Path
import enum
from typing import Tuple
import numpy as np
import pickle
from sklearn.model_selection import train_test_split
from sklearn.gaussian_process import GaussianProcessClassifier
from sklearn.gaussian_process.kernels import RBF
from rospkg import RosPack


class Location(enum.Enum):
    """
    Enum containomg locations of interest (i.e. where any of the requested objects may be found.)
    """
    Cupboard = 0
    CupboardTop = 1
    Refrigerator = 2
    Table = 3

class Object(enum.Enum):
    """
    Enum containing all subclasses of RequestableObjects
    """
    Bread = 0
    EdibleFruit = 1
    CitrusFruit = 2
    Milk = 3
    EatingVessel = 4
    DrinkingVessel = 5

class UsageContext(enum.Enum):  # scenes to be used
    """
    Enum for different cases the object can be used for. Can affect prediction outcome.
    """
    Breakfast = 0
    Lunch = 1
    Dinner = 2
    Snack = 3
    Party = 4

class ObjectLocator():
    """
    Class that contains the GPC.
    """

    # Map object to a mean weight and size. Used to improve prediction
    object_properties = {
        Object.Bread.value: {"weight": 0.5, "size": 0.5},
        Object.EdibleFruit.value: {"weight": 0.2, "size": 0.4},
        Object.CitrusFruit.value: {"weight": 0.15, "size": 0.35},
        Object.Milk.value: {"weight": 1.0, "size": 0.8},
        Object.EatingVessel.value: {"weight": 0.4, "size": 0.15},
        Object.DrinkingVessel.value: {"weight": 0.3, "size": 0.2},
    }


    # Probabilistic relationship between items and locations. Used to generate train/test dataset.
    object_to_location = {
        Object.EdibleFruit.value: [(Location.Refrigerator.value, 0.8), (Location.Table.value, 0.2)],
        Object.CitrusFruit.value: [(Location.Refrigerator.value, 0.8), (Location.Table.value, 0.2)],
        Object.Bread.value: [(Location.Cupboard.value, 0.5), (Location.Table.value, 0.3), (Location.Refrigerator.value, 0.2)],
        Object.Milk.value: [(Location.Refrigerator.value, 1.0)],  # Assuming milk is always in the refrigerator
        Object.EatingVessel.value: [(Location.Table.value, 0.55), (Location.CupboardTop.value, 0.45)],
        Object.DrinkingVessel.value: [(Location.CupboardTop.value, 0.75), (Location.Table.value, 0.25)],
    }

    def __init__(self, train = False, N = 150):
        #  Get dataset
        X, y = self.convert_dict_to_data(N)
        # Split dataset to train and test.
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3)

        # GPC variable
        self.classifier = None

        # use rospack to get path of this ros package.
        rp = RosPack()
        model_file_path = rp.get_path('bayesian_object_finder') +"/classifying_model/"
        model_file_name = Path("classifier")
        model_full_path = str(model_file_path / model_file_name)

        # Check if a trained model is saved at the provided path.
        model_saved = os.path.exists(model_full_path)

        # Model parameters are not saved or training wanted.
        if not model_saved or train:
            if not model_saved:
                print("No model is saved at %s, training a new one."%model_full_path)
            else:
                print("Training new model.")

            # define, train, and test the model. Then save the resulting model.
            self.classifier = GaussianProcessClassifier(1.0 * RBF(1.0), n_jobs = 16)
            self.classifier.fit(X_train, y_train)
            accuracy = self.classifier.score(X_test, y_test)
            print("Model Accuracy:", accuracy)
            pickle.dump(self.classifier, open(model_full_path, 'wb'))
        else:
            # load in, and then test the saved model.
            print("Found a pre-trained model.")
            self.classifier: GaussianProcessClassifier = pickle.load(open(model_full_path, 'rb'))
            accuracy = self.classifier.score(X_test, y_test)
            print("Model Accuracy:", accuracy)

    def convert_dict_to_data(self, N: int = 100) -> Tuple[np.array, np.array]:
        """
        Converts the map object_to_location to a dataset containing instances of objects being
        found at the provided locations with the given proportionalities.
        Easy way to generate train/test data.

        Args:
            N (int, optional): Total number of datapoints for each object. Defaults to 100.

        Returns:
            Tuple[np.array, np.array]: returns array of objects, size/weight, usage context (X),
            and their corresponding locations (y). Objects/Locations are stored as value of
            their respective enums.
        """
        X, y = [], []
        for object_type, possible_locations in ObjectLocator.object_to_location.items():
            properties = ObjectLocator.object_properties[object_type]
            for _ in range(N):
                weight = properties["weight"] + np.random.normal(0, 0.05)
                size = properties["size"] + np.random.normal(0, 0.05)
                context: UsageContext = np.random.choice(list(UsageContext))
                # Make sure object_type is an enumeration member
                X.append([object_type, weight, size, context.value])

                labels, weights = zip(*possible_locations)
                location = np.random.choice(labels, p=weights)
                y.append(location)

        return np.array(X), np.array(y)

    def __call__(self, x: np.array) -> np.array:
        """
        Make prediction using GPC

        Args:
            x (np.array): Input to GPC (object, weight/size, usage context)

        Returns:
            np.array: array of probabilities, ordered same as Location enum.
        """
        if type(x) == str:
            weight_and_size = list(ObjectLocator.object_properties[Object[x].value].values())
            context = np.random.choice(list(UsageContext))
            x = np.array([[Object[x].value, weight_and_size[0], weight_and_size[1], context.value]])
        probs = self.classifier.predict_proba(x)

        return probs

