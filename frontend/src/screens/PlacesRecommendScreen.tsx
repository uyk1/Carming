import {SafeAreaView} from 'react-native-safe-area-context';
import LinearGradient from 'react-native-linear-gradient';
import styled from 'styled-components/native';
import CommonChip from '../components/CommonChip';
import {IconButton, useTheme} from 'react-native-paper';
import {SegmentedButtons} from 'react-native-paper';
import {useState} from 'react';

const PlacesRecommendScreen = () => {
  const theme = useTheme();

  const [recommendType, setRecommendType] = useState<string>('');
  const [checkedTagIdList, setCheckedTagIdList] = useState<number[]>([]);

  const recommendTypeChangeButtons = [
    {
      value: '0',
      label: '장소',
      icon: 'map-marker',
      checkedColor: 'white',
      uncheckedColor: 'white',
      style: {
        borderRadius: 10,
        backgroundColor:
          recommendType === '0' ? theme.colors.primary : theme.colors.shadow,
      },
    },
    {
      value: '1',
      label: '코스',
      icon: 'routes',
      checkedColor: 'white',
      uncheckedColor: 'white',
      style: {
        borderRadius: 10,
        backgroundColor:
          recommendType === '1' ? theme.colors.primary : theme.colors.shadow,
      },
    },
  ];

  const recommendTagList: Tag[] = [
    {
      id: 0,
      text: '맛있는',
    },
    {
      id: 1,
      text: '청결한',
    },
    {
      id: 2,
      text: '유명한',
    },
  ];

  const tagPressed = (tagId: number) => {
    checkedTagIdList.includes(tagId)
      ? checkedTagIdList.splice(checkedTagIdList.indexOf(tagId), 1)
      : checkedTagIdList.push(tagId);
    setCheckedTagIdList([...checkedTagIdList]);
  };

  return (
    <GradientBackground colors={['#70558e7a', '#df94c283', '#ffbdc1b0']}>
      <StyledSafeAreaView style={{justifyContent: 'space-between'}}>
        <SegmentedButtons
          style={{width: 200}}
          value={recommendType}
          onValueChange={setRecommendType}
          buttons={recommendTypeChangeButtons}
        />
        <IconButton
          icon="home"
          size={25}
          onPress={() => {
            console.log('hello');
          }}
        />
      </StyledSafeAreaView>
      <StyledSafeAreaView style={{marginTop: 10}}>
        {recommendTagList.map(element => {
          return (
            <CommonChip
              key={element.id}
              style={{marginLeft: 5}}
              text={element.text}
              selected={checkedTagIdList.includes(element.id)}
              selectedBackgroundColor={theme.colors.secondary}
              onPress={() => tagPressed(element.id)}
            />
          );
        })}
      </StyledSafeAreaView>
    </GradientBackground>
  );
};

type Tag = {
  id: number;
  text: string;
};

const StyledSafeAreaView = styled(SafeAreaView)`
  align-items: center;
  flex-direction: row;
`;

const GradientBackground = styled(LinearGradient)`
  flex: 1;
  padding: 20px;
`;

export default PlacesRecommendScreen;
