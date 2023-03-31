import {Coordinate, Course, Place} from '../types';
import MapView from 'react-native-maps';
import {calcCoordinates} from '../utils';
import MapMarker from './MapMarker';
import MapPolyline from './MapPolyline';
import {ViewStyle} from 'react-native/types';

interface CustomMapViewProps {
  course?: Course;
  places?: Place[];
  viewStyle?: ViewStyle;
  routeCoordinates?: Coordinate[];
}

const CustomMapView: React.FC<CustomMapViewProps> = ({
  course,
  places,
  viewStyle,
  routeCoordinates,
}) => {
  const placeList = course ? course.places : places ? places : [];
  const coordinates = placeList.map<Coordinate>(place => {
    return {latitude: place.lat, longitude: place.lon};
  });
  const {midLat, midLon, latDelta, lonDelta} = calcCoordinates(coordinates);
  return (
    <MapView
      style={viewStyle}
      initialRegion={{
        latitude: midLat + 0.2 * latDelta,
        longitude: midLon,
        latitudeDelta: latDelta,
        longitudeDelta: lonDelta,
      }}>
      <MapPolyline coordinates={routeCoordinates ?? []} />
      {placeList.map((place, index) => (
        <MapMarker key={place.id} place={place} index={index} />
      ))}
    </MapView>
  );
};

export default CustomMapView;
