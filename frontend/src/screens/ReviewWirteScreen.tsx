import {SafeAreaView, StyleSheet, Text, View} from 'react-native';
import {CustomButton, ReviewCourseItem, ReviewPlaceItem} from '../components';
import {useDispatch, useSelector} from 'react-redux';
import {RootState} from '../redux/store';
import {filterRealPlace, placesToPlaceReviewRequests} from '../utils';
import {
  Category,
  Course,
  CourseReviewRequest,
  Place,
  PlaceReviewRequest,
} from '../types';
import {useEffect, useState} from 'react';
import {useGetTagsQuery} from '../apis/tagApi';
import {setTagList} from '../redux/slices/tagSlice';
import styled from 'styled-components';
import {setCourseReview, setplaceReviews} from '../redux/slices/reviewSlice';
import {useTheme} from 'react-native-paper';

const ReviewWirteScreen = () => {
  const theme = useTheme();
  const dispatch = useDispatch();
  const placeList = useSelector((state: RootState) =>
    filterRealPlace(state.journey.placeList),
  );
  const {placeReviews, courseReview, courseTitle} = useSelector(
    (state: RootState) => state.review,
  );

  useEffect(() => {
    dispatch(setplaceReviews(placesToPlaceReviewRequests(placeList)));
    dispatch(setCourseReview({...courseReview, courseId: 10}));
  }, []);

  const {data: tagLists} = useGetTagsQuery();

  useEffect(() => {
    if (tagLists !== undefined) dispatch(setTagList(tagLists));
  }, [tagLists]);

  return (
    <StyledSafeAreaView>
      <ScreenContainer>
        <TitleText style={{color: theme.colors.surfaceVariant}}>
          리뷰 작성하기
        </TitleText>
        {placeList.map((place, index) => (
          <ReviewPlaceItem key={place.id} place={place} index={index + 1} />
        ))}
        <ReviewCourseItem isFirstTime={false} />
        <CustomButton
          text={'완료'}
          onPress={() => {}}
          buttonStyle={{
            ...styles.button,
            backgroundColor: theme.colors.surfaceVariant,
          }}
          textStyle={styles.buttonText}
        />
      </ScreenContainer>
    </StyledSafeAreaView>
  );
};

const dummyPlace: Place = {
  id: 4,
  name: '카페공명',
  tel: '070-8869-6304',
  category: Category.카페,
  lon: 126.92635262,
  lat: 37.5598709,
  region: '마포구',
  ratingCount: 127,
  ratingSum: 470,
  keyword: ['공부하기좋은', '베이커리카페', '북카페', '이색카페'],
  image:
    'https://d1ldvf68ux039x.cloudfront.net/thumbs/frames/audio/2211/71249/122x92_q95.jpg',
};
const dummyCourse: Course = {
  name: '노잼은 아닌 코스',
  regions: ['노원구', '은평구', '관악구'],
  places: [
    {
      id: 1,
      name: '어반플랜트 합정',
      lon: 126.9171881,
      lat: 37.54789417,
      image: 'http://t1.daumcdn.net/localfiy/searchregister_1152174062',
      ratingCount: 275,
      ratingSum: 1155,
    },
    {
      id: 2,
      name: '피오니 홍대점',
      lon: 126.91976584,
      lat: 37.55008538,
      image: 'http://t1.daumcdn.net/cfile/276E0A4C550A37652A',
      ratingCount: 214,
      ratingSum: 813,
    },
    {
      id: 3,
      name: '프릳츠 도화점',
      lon: 126.94907049,
      lat: 37.54101958,
      image:
        'http://t1.kakaocdn.net/fiy_reboot/place/0E03C7048DF94AB187582A216C7CA500',
      ratingCount: 288,
      ratingSum: 1123,
    },
    {
      id: 4,
      name: '카페공명',
      lon: 126.92635262,
      lat: 37.5598709,
      image: 'http://t1.daumcdn.net/localfiy/searchregister_1919726515',
      ratingCount: 127,
      ratingSum: 470,
    },
  ],
};

const styles = StyleSheet.create({
  button: {
    width: 200,
    height: 50,
    padding: 14,
    borderRadius: 30,
    marginTop: 40,
  },
  buttonText: {
    fontWeight: 'bold',
    fontSize: 16,
    textAlign: 'center',
  },
});

const StyledSafeAreaView = styled(SafeAreaView)`
  flex: 1;
  background-color: white;
`;

const ScreenContainer = styled(View)`
  flex: 1;
  flex-direction: column;
  align-items: center;
  padding: 20px;
`;

const TitleText = styled(Text)`
  font-size: 22px;
  font-weight: bold;
  width: 100%;
  margin: 20px 0px 40px 0px;
`;

export default ReviewWirteScreen;
