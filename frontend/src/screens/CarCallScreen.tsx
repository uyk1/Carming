import {CompositeScreenProps} from '@react-navigation/native';
import {NativeStackScreenProps} from '@react-navigation/native-stack';
import {useState} from 'react';
import {StyleSheet, Dimensions} from 'react-native';
import {useTheme} from 'react-native-paper';
import {SafeAreaView} from 'react-native-safe-area-context';
import styled from 'styled-components';
import {CarCallInfoCard, CustomButton, CustomMapView} from '../components';
import {iconPlace} from '../components/MapMarker';
import {L3_TotalJourneyStackParamList} from '../navigations/L3_TotalJourneyStackNavigator';
import {L4_JourneyStackParamList} from '../navigations/L4_JourneyStackNavigator';
import {placesToCoordinates} from '../utils';

export type CarCallScreenProps = CompositeScreenProps<
  NativeStackScreenProps<L4_JourneyStackParamList, 'CarCall'>,
  NativeStackScreenProps<L3_TotalJourneyStackParamList>
>;

const iconPlaces: iconPlace[] = [
  {
    iconName: 'hail',
    lon: 126.97944891,
    lat: 37.57171765,
  },
  {
    iconName: 'taxi',
    lon: 126.97844891,
    lat: 37.57271765,
  },
  {
    iconName: 'map-marker',
    lon: 126.97744891,
    lat: 37.57371765,
  },
];

const CarCallScreen: React.FC<CarCallScreenProps> = ({navigation, route}) => {
  const theme = useTheme();
  const routeCoordinates = placesToCoordinates(iconPlaces);
  const [buttonAbled, setButtonAbled] = useState<boolean>(true);

  return (
    <StyledSafeAreaView>
      <CarCallInfoCard
        statusText={'카밍카가 출발했어요 '}
        infoText={'14분 내로 도착할 예정이에요 '}
      />
      <CustomMapView
        places={iconPlaces}
        viewStyle={{flex: 1}}
        latitudeOffset={0}
        routeCoordinates={routeCoordinates}
      />
      <CustomButton
        text={'차량 탑승 완료'}
        onPress={() => {}}
        disabled={!buttonAbled}
        buttonStyle={{
          ...styles.buttonStyle,
          backgroundColor: buttonAbled
            ? theme.colors.secondary
            : theme.colors.surfaceDisabled,
        }}
        textStyle={styles.buttonText}
      />
    </StyledSafeAreaView>
  );
};

const styles = StyleSheet.create({
  buttonStyle: {
    width: 200,
    padding: 14,
    borderRadius: 30,
    position: 'absolute',
    bottom: 20,
    left: Dimensions.get('window').width * 0.5 - 100,
  },
  buttonText: {
    fontWeight: 'bold',
    fontSize: 14,
    textAlign: 'center',
  },
});

const StyledSafeAreaView = styled(SafeAreaView)`
  flex: 1;
  justify-content: center;
`;

export default CarCallScreen;
