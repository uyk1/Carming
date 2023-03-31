import {CompositeScreenProps} from '@react-navigation/native';
import {NativeStackScreenProps} from '@react-navigation/native-stack';
import {useState} from 'react';
import {StyleSheet, View} from 'react-native';
import {useTheme} from 'react-native-paper';
import {SafeAreaView} from 'react-native-safe-area-context';
import {useSelector} from 'react-redux';
import styled from 'styled-components';
import {CarMoveInfoCard, CustomButton, CustomMapView} from '../components';
import {L3_TotalJourneyStackParamList} from '../navigations/L3_TotalJourneyStackNavigator';
import {L4_JourneyStackParamList} from '../navigations/L4_JourneyStackNavigator';
import {RootState} from '../redux/store';
import {Category, Place} from '../types';
import {placesToCoordinates} from '../utils';

export type CarMoveScreenProps = CompositeScreenProps<
  NativeStackScreenProps<L4_JourneyStackParamList, 'CarMove'>,
  NativeStackScreenProps<L3_TotalJourneyStackParamList>
>;

const CarMoveScreen: React.FC<CarMoveScreenProps> = ({navigation, route}) => {
  const theme = useTheme();

  const routeCoordinates = placesToCoordinates(places);
  const [currentPlaceIdx, setCurrentPlaceIdx] = useState<number>(0);
  const [buttonAbled, setButtonAbled] = useState<boolean>(false);

  return (
    <StyledSafeAreaView>
      <CustomMapView
        places={places}
        viewStyle={{flex: 1}}
        latitudeOffset={0.2}
        routeCoordinates={routeCoordinates}
      />
      {}
      <CarMoveInfoCard
        place={places[1]}
        index={1}
        infoText={'도착까지 27분 남았어요 '}
      />
      <ButtonContainer>
        <CustomButton
          text={'하차 완료'}
          onPress={() => {}}
          disabled={!buttonAbled}
          buttonStyle={{
            ...styles.buttonStyle,
            backgroundColor: buttonAbled
              ? theme.colors.surfaceVariant
              : theme.colors.surfaceDisabled,
          }}
          textStyle={styles.buttonText}
        />
        <CustomButton
          text={'잠시 정차하기'}
          onPress={() => {}}
          disabled={buttonAbled}
          buttonStyle={{
            ...styles.buttonStyle,
            backgroundColor: buttonAbled
              ? theme.colors.surfaceDisabled
              : theme.colors.tertiary,
          }}
          textStyle={styles.buttonText}
        />
      </ButtonContainer>
    </StyledSafeAreaView>
  );
};

const styles = StyleSheet.create({
  buttonStyle: {
    width: 150,
    padding: 14,
    borderRadius: 30,
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

const ButtonContainer = styled(View)`
  flex-direction: row;
  align-items: center;
  justify-content: center;
  margin-bottom: 20px;
`;

const places: Place[] = [
  {
    id: 0,
    name: '허니치즈 순대국',
    image: 'https://i.imgur.com/UYiroysl.jpg',
    ratingSum: 17,
    ratingCount: 4,
    region: '노원구 중계 14동',
    lon: 126.97944891,
    lat: 37.57171765,
    tel: '010-1577-1577',
    category: Category.음식점,
    keyword: ['맛있는', '분위기 좋은'],
  },
  {
    id: 1,
    name: '파리 엉터리 생고기',
    image: 'https://i.imgur.com/UPrs1EWl.jpg',
    ratingSum: 222,
    ratingCount: 80,
    region: '노원구 중계 14동',
    lon: 126.98197125,
    lat: 37.58459777,
    tel: '010-1577-1577',
    category: Category.음식점,
    keyword: ['맛있는', '분위기 좋은'],
  },
];

export default CarMoveScreen;
