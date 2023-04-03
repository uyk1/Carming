import {useState} from 'react';
import {
  Modal,
  Text,
  TouchableOpacity,
  View,
  TextInput,
  Alert,
  ImageBackground,
  ScrollView,
  StyleSheet,
} from 'react-native';
import {useVerifyMutation} from '../apis/memberRegistApi';
import {useDispatch, useSelector} from 'react-redux';
import {verifySuccess} from '../redux/slices/authSlice';
import {Course, Place} from '../types';
import styled from 'styled-components';
import RatingStar from './RatingStar';
import {calcRating} from '../utils';
import SIcon from 'react-native-vector-icons/SimpleLineIcons';
import MIcon from 'react-native-vector-icons/MaterialIcons';
import {addPlaceToPlaceCart} from '../redux/slices/placeSlice';
import {
  addPlaceListToPreCart,
  addPlaceToPreCart,
} from '../redux/slices/mainSlice';
import {RootState} from '../redux/store';
import CustomMapView from './CustomMapView';
import CustomButton from './CustomButton';

export interface MainCourseCardModalProps {
  isVisible: boolean;
  onClose: () => void;
  course: Course;
}

const MainCourseCardModal: React.FC<MainCourseCardModalProps> = ({
  isVisible,
  onClose,
  course,
}) => {
  const dispatch = useDispatch();

  const rating = calcRating(course.ratingSum, course.ratingCount);

  const preCart = useSelector((state: RootState) => state.main.preCart);

  const handleCancel = () => {
    onClose();
  };

  const handleAddPress = () => {
    dispatch(addPlaceListToPreCart(course.places));
  };

  return (
    <Modal
      animationType="fade"
      transparent={true}
      visible={isVisible}
      onRequestClose={onClose}>
      <BackgroundView>
        <ModalContainer>
          <ContentsContainer>
            <ImgView>
              <CustomMapView
                viewStyle={{flex: 1}}
                places={course.places}
                useIndex={true}
              />
            </ImgView>
            <ContentView
              style={{
                alignItems: 'center',
                justifyContent: 'space-between',
                marginTop: '3%',
                marginBottom: '8%',
              }}>
              <CustomText
                style={{
                  fontFamily: 'SeoulNamsanM',
                  fontSize: 14,
                  marginVertical: '1%',
                }}>
                {course.name}
              </CustomText>
              <View style={{flexDirection: 'row', alignItems: 'center'}}>
                <RatingStar
                  rating={rating}
                  containerStyle={{marginRight: 8}}
                  iconStyle={{margin: -8}}
                  inactiveColor="grey"
                />
                <RatingText>
                  {rating} ({course.ratingCount})
                </RatingText>
              </View>
            </ContentView>
            <ContentView>
              <ScrollView
                horizontal
                nestedScrollEnabled={true}
                showsHorizontalScrollIndicator={false}
                style={styles.textContainer}>
                {course.places.map((place, index) => (
                  <Text key={index} style={[styles.text]}>
                    {place.name + ' - '}
                  </Text>
                ))}
              </ScrollView>
            </ContentView>
            <ContentView>
              <CustomText></CustomText>
            </ContentView>
          </ContentsContainer>
          <View
            style={{
              flexDirection: 'row',
              justifyContent: 'flex-end',
              gap: 10,
            }}>
            <TouchableOpacity
              onPress={handleAddPress}
              style={{
                backgroundColor: 'black',
                paddingHorizontal: 15,
                padding: 7,
                borderRadius: 5,
                alignItems: 'center',
                // opacity: isIncluded ? 0.5 : 1, // disabled 상태일 때 투명도 설정
              }}>
              <Text style={{fontFamily: 'SeoulNamsanM', color: '#fff'}}>
                지금 출발
              </Text>
            </TouchableOpacity>
            <TouchableOpacity
              onPress={handleCancel}
              style={{
                backgroundColor: '#a5a5a5',
                paddingHorizontal: 15,
                padding: 7,
                borderRadius: 5,
                alignItems: 'center',
              }}>
              <Text style={{fontFamily: 'SeoulNamsanM', color: '#fff'}}>
                나가기
              </Text>
            </TouchableOpacity>
          </View>
        </ModalContainer>
      </BackgroundView>
    </Modal>
  );
};

const BackgroundView = styled(View)`
  flex: 1;
  justify-content: center;
  align-items: center;
  background-color: rgba(0, 0, 0, 0.5);
`;

const ModalContainer = styled(View)`
  background-color: rgba(255, 255, 255, 0.8);
  height: 70%;
  width: 85%;
  border-radius: 3px;
  padding: 5%;
  justify-content: space-between;
`;

const ContentsContainer = styled(View)`
  flex: 1;
`;
const ContentView = styled(View)`
  flex-direction: row;
  align-items: center;
  margin-bottom: 4%;
`;

const ImgView = styled(View)`
  height: 55%;
  width: 100%;
  border-radius: 3px;
  overflow: hidden;
  margin-bottom: 3%;
`;
const PlaceImg = styled(ImageBackground)`
  height: 100%;
  width: 100%;
`;
const CustomText = styled(Text)`
  font-family: SeoulNamsanB;
  font-size: 12px;
`;
const RatingText = styled(Text)`
  color: black;
  font-size: 13px;
`;

const styles = StyleSheet.create({
  textContainer: {
    flexDirection: 'row',
    alignContent: 'center',
  },
  text: {
    fontFamily: 'SeoulNamsanM',
    color: 'black',
    fontSize: 20,
  },
});
export default MainCourseCardModal;
